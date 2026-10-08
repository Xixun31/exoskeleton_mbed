// =====================================================================
// 最多 5 顆 SPI IMU (Xsens MTi，共用一組 SPI) + 1 個 SPI encoder (另一組 SPI)
// 以固定頻率 (預設 500Hz，可切換) 同步讀取，檢查時間同步性與是否漏讀 / 調包
// IMU 本身以 100Hz 輸出；MCU 讀得比較快，每個新封包最多延遲一個讀取週期就會被讀到
//
// 序列埠 921600 baud。指令：
//   'r' : 重新初始化 IMU，印出每顆的 INIT 結果 (DeviceID 等)
//   'g' : 開始串流資料
//   'x' : 停止串流
//   '1'/'2'/'3' : IMU SPI 時脈切換為 200k / 500k / 1M (之後要再送 'r' 重新初始化)
//   'b' : 匯流排檢查：所有 CS 都不選取時送 ProtInfo，正常應該沒有 IMU 回應 FA FF FF FF；
//         再逐顆單獨選取，確認每條 CS 只控制到一顆
//   'A'/'B'/'C'/'D'/'E' : 讀取頻率切換為 100 / 200 / 400 / 500 / 1000 Hz
//   'p' : 時間校準：2 秒內盡快輪詢 4 顆 IMU，印出每個新封包的到達時間
//         P,imu,t_us,pc,stf  (用來把 SampleTimeFine 換算成 MCU 時間)
//
// 串流格式 (每個讀取週期一行，只輸出初始化成功的 IMU)：
//   D,cycle,t_us,enc_off,enc_deg,enc_err,K,
//     [K 顆 IMU 各 8 欄] idx,off,new,bad,pc,stf,pitch,gy,
//   busy,dropped
//   busy    : 這個週期從開始到讀完所有裝置花的時間 (us)，必須小於週期
//   t_us    : 這個週期開始的 MCU 時間 (us)
//   *_off   : 該裝置開始被讀取的時間 - 週期開始 (us)
//   new     : 這個週期讀到幾個新封包 (0 = 沒有新資料, >1 = 有積壓)
//   bad     : 格式 / checksum 錯誤的讀取次數
//   pc      : 最新封包的 PacketCounter
//   stf     : 最新封包的 SampleTimeFine (IMU 內部時間，單位 0.1 ms)
//   pitch   : 最新封包的 pitch (deg)
//   gy      : 最新封包的 gyro y (deg/s)
//   dropped : 因序列埠來不及送出而丟掉的週期數 (累計)
// =====================================================================
#include "mbed.h"
#include <cmath>

// ---------------- 接線設定 ----------------
// IMU：共用 SPI2 (Morpho)  MOSI PB_15, MISO PB_14, SCLK PB_13
SPI spi_imu(PB_15, PB_14, PB_13);
int imu_spi_freq = 1000000;  // 4 顆 x 100Hz 至少要約 500kHz；送 '1'/'2'/'3' 切換 200k/500k/1M
const int N_IMU = 5;  // 沒接的 CS 會在初始化時自動略過
DigitalOut imu_cs[N_IMU] = {DigitalOut(PB_1, 1), DigitalOut(PB_2, 1), DigitalOut(PB_12, 1),
                            DigitalOut(PC_5, 1), DigitalOut(PC_4, 1)};
const char* IMU_CS_NAME[N_IMU] = {"PB_1", "PB_2", "PB_12", "PC_5", "PC_4"};

// Encoder：另一組 SPI (Arduino D3/D4/D5，與 encoder/main.cpp 相同)
SPI spi_enc(D4, D5, D3);  // mosi, miso, sclk
DigitalOut enc_cs(D9, 1);
const int ENC_SPI_FREQ = 1000000;

volatile int rate_hz = 500;  // 讀取頻率，用 'A'~'E' 切換
const float RAD2DEG = 57.2957795f;

// ---------------- MTi 協定 ----------------
const uint8_t OP_PROTINFO = 0x01, OP_CONFIGPROT = 0x02, OP_CONTROL = 0x03, OP_PIPESTATUS = 0x04,
              OP_NOTIFICATION = 0x05, OP_MEASUREMENT = 0x06;
const uint8_t MID_REQDID = 0x00, MID_DEVICEID = 0x01, MID_GOTOMEAS = 0x10, MID_GOTOMEAS_ACK = 0x11,
              MID_GOTOCONFIG = 0x30, MID_GOTOCONFIG_ACK = 0x31, MID_MTDATA2 = 0x36;

uint8_t fillword[4];
uint8_t rx[256];
bool verbose = false;  // 'v' 切換：初始化時印出每個 notification

void imu_transfer(int i, uint8_t op, const uint8_t* tx, uint8_t* out, int len) {
    imu_cs[i] = 0;
    fillword[0] = spi_imu.write(op);
    for (int k = 0; k < 3; k++) {
        fillword[k + 1] = spi_imu.write(k);
    }
    for (int k = 0; k < len; k++) {
        uint8_t b = spi_imu.write(tx ? tx[k] : 0x00);
        if (out) out[k] = b;
    }
    imu_cs[i] = 1;
    wait_us(5);
}

void pipe_status(int i, uint16_t* n, uint16_t* m) {
    uint8_t b[4];
    imu_transfer(i, OP_PIPESTATUS, nullptr, b, 4);
    *n = b[0] | (b[1] << 8);
    *m = b[2] | (b[3] << 8);
}

bool checksum_ok(const uint8_t* msg, int len) {
    uint8_t sum = 0xFF;
    for (int k = 0; k < len; k++) sum += msg[k];
    return sum == 0;
}

void send_xbus(int i, uint8_t mid) {
    uint8_t msg[3] = {mid, 0x00, (uint8_t)(-(uint8_t)(0xFF + mid))};
    imu_transfer(i, OP_CONTROL, msg, nullptr, 3);
}

int wait_notification(int i, uint8_t want, uint8_t* payload, int timeout_ms) {
    Timer t;
    t.start();
    while (t.elapsed_time() < std::chrono::milliseconds(timeout_ms)) {
        uint16_t n, m;
        pipe_status(i, &n, &m);
        if (verbose && (n || m)) printf("  [v] imu%d want %02X: notif=%u meas=%u\r\n", i, want, n, m);
        if (n > 0 && n <= sizeof(rx)) {
            imu_transfer(i, OP_NOTIFICATION, nullptr, rx, n);
            if (verbose) {
                printf("  [v]   notif:");
                for (int k = 0; k < n && k < 12; k++) printf(" %02X", rx[k]);
                printf(" cksum=%d\r\n", checksum_ok(rx, n));
            }
            if (rx[0] == want && checksum_ok(rx, n)) {
                if (payload) memcpy(payload, &rx[2], rx[1]);
                return rx[1];
            }
        } else if (m > 0 && m <= sizeof(rx)) {
            imu_transfer(i, OP_MEASUREMENT, nullptr, rx, m);
        }
        ThisThread::sleep_for(2ms);
    }
    return -1;
}

float be_float(const uint8_t* p) {
    uint32_t v = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
    float f;
    memcpy(&f, &v, 4);
    return f;
}

bool imu_ok[N_IMU];

void init_imus() {
    for (int i = 0; i < N_IMU; i++) {
        uint8_t info[2];
        imu_transfer(i, OP_PROTINFO, nullptr, info, 2);
        bool prot = fillword[0] == 0xFA && fillword[1] == 0xFF && fillword[2] == 0xFF && fillword[3] == 0xFF;
        uint32_t did = 0;
        bool meas = false;
        if (prot) {
            const uint8_t cfg = 0x08;
            imu_transfer(i, OP_CONFIGPROT, &cfg, nullptr, 1);
            send_xbus(i, MID_GOTOCONFIG);
            wait_notification(i, MID_GOTOCONFIG_ACK, nullptr, 500);
            uint8_t d[8];
            send_xbus(i, MID_REQDID);
            if (wait_notification(i, MID_DEVICEID, d, 500) == 4) {
                did = ((uint32_t)d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3];
            }
            send_xbus(i, MID_GOTOMEAS);
            meas = wait_notification(i, MID_GOTOMEAS_ACK, nullptr, 500) >= 0;
        }
        imu_ok[i] = prot && did != 0;
        printf("INIT,%d,%s,fw=%02X%02X%02X%02X,did=%08lX,meas=%d,ok=%d\r\n", i, IMU_CS_NAME[i], fillword[0],
               fillword[1], fillword[2], fillword[3], (unsigned long)did, meas, imu_ok[i]);
    }
    printf("INIT_DONE,spi=%d,rate=%d\r\n", imu_spi_freq, rate_hz);
}

// ---------------- 每週期的資料 ----------------
struct ImuSample {
    uint16_t off;  // 開始讀取的時間 - 週期開始 (us)
    uint8_t n_new, bad;
    uint16_t pc;
    uint32_t stf;
    float pitch, gy;
};
struct Cycle {
    uint32_t cycle, t_us;
    uint16_t enc_off;
    float enc_deg;
    uint8_t enc_err;
    uint16_t busy;
    ImuSample imu[N_IMU];
};
Mail<Cycle, 64> mail;
volatile uint32_t dropped = 0;
volatile bool streaming = false;
Timer clock_us;

// 解析一段 measurement pipe 資料 (可能含多個 MTData2 訊息)，更新最新值
void parse_measurement(const uint8_t* buf, int len, ImuSample& s) {
    int off = 0;
    while (off + 3 <= len) {
        int mlen = buf[off + 1] + 3;
        if (buf[off] != MID_MTDATA2 || off + mlen > len || !checksum_ok(&buf[off], mlen)) {
            s.bad++;
            return;
        }
        s.n_new++;
        int i = off + 2, end = off + 2 + buf[off + 1];
        while (i + 3 <= end) {
            uint16_t xdi = (buf[i] << 8) | buf[i + 1];
            int size = buf[i + 2];
            const uint8_t* c = &buf[i + 3];
            if (i + 3 + size > end) break;
            switch (xdi & 0xFFF0) {
                case 0x1020: if (size == 2) s.pc = (c[0] << 8) | c[1]; break;
                case 0x1060: if (size == 4) s.stf = ((uint32_t)c[0] << 24) | (c[1] << 16) | (c[2] << 8) | c[3]; break;
                case 0x2030: if (size == 12) s.pitch = be_float(c + 4); break;
                case 0x8020: if (size == 12) s.gy = be_float(c + 4) * RAD2DEG; break;
            }
            i += 3 + size;
        }
        off += mlen;
    }
}

void read_imu(int i, uint32_t t0, ImuSample& s) {
    s.off = (uint16_t)(clock_us.elapsed_time().count() - t0);
    if (!imu_ok[i]) return;
    for (int guard = 0; guard < 4; guard++) {  // 把積壓的封包都讀完
        uint16_t n, m;
        pipe_status(i, &n, &m);
        if (n > 0 && n <= sizeof(rx)) {
            imu_transfer(i, OP_NOTIFICATION, nullptr, rx, n);
        }
        if (m == 0) break;
        if (m > sizeof(rx)) {
            s.bad++;
            break;
        }
        imu_transfer(i, OP_MEASUREMENT, nullptr, rx, m);
        parse_measurement(rx, m, s);
    }
}

void read_encoder(uint32_t t0, Cycle& c) {
    c.enc_off = (uint16_t)(clock_us.elapsed_time().count() - t0);
    enc_cs = 0;
    wait_us(1);
    uint16_t raw = spi_enc.write(0x0000);
    enc_cs = 1;
    c.enc_err = (raw & 0x8000) ? 0 : 1;  // bit15 恆為 1
    c.enc_deg = ((raw >> 3) & 0x0FFF) / 4096.0f * 360.0f;
}

Thread sample_thread(osPriorityRealtime, 4096);
Ticker sample_ticker;  // us 解析度 (RTOS sleep 只有 1ms 解析度，做不到 400Hz)
EventFlags sample_flag;

void on_tick() { sample_flag.set(1); }

void set_rate(int hz) {
    rate_hz = hz;
    sample_ticker.detach();
    sample_ticker.attach(callback(on_tick), std::chrono::microseconds(1000000 / hz));
}

void sample_loop() {
    uint32_t cycle = 0;
    ImuSample last[N_IMU] = {};
    while (true) {
        sample_flag.wait_any(1);
        if (streaming) {
            Cycle c = {};
            c.cycle = cycle++;
            uint32_t t0 = clock_us.elapsed_time().count();
            c.t_us = t0;
            read_encoder(t0, c);
            for (int i = 0; i < N_IMU; i++) {
                ImuSample s = last[i];  // 沒有新封包時保留上一筆的值
                s.n_new = 0;
                s.bad = 0;
                read_imu(i, t0, s);
                c.imu[i] = s;
                last[i] = s;
            }
            c.busy = (uint16_t)(clock_us.elapsed_time().count() - t0);
            Cycle* p = mail.try_alloc();
            if (p) {
                *p = c;
                mail.put(p);
            } else {
                dropped++;
            }
        }
    }
}

void bus_check() {
    // 1. 全部不選取
    for (int rep = 0; rep < 3; rep++) {
        uint8_t b[6];
        fillword[0] = spi_imu.write(OP_PROTINFO);
        for (int k = 0; k < 3; k++) fillword[k + 1] = spi_imu.write(k);
        for (int k = 0; k < 6; k++) b[k] = spi_imu.write(0);
        printf("BUS,none,%02X%02X%02X%02X,%02X%02X%02X%02X%02X%02X\r\n", fillword[0], fillword[1], fillword[2],
               fillword[3], b[0], b[1], b[2], b[3], b[4], b[5]);
        wait_us(100);
    }
    // 2. 逐顆單獨選取，讀 PipeStatus (每顆的值通常不同)
    for (int i = 0; i < N_IMU; i++) {
        for (int rep = 0; rep < 3; rep++) {
            uint8_t b[4];
            imu_transfer(i, OP_PIPESTATUS, nullptr, b, 4);
            printf("BUS,%d,%02X%02X%02X%02X,%02X%02X%02X%02X\r\n", i, fillword[0], fillword[1], fillword[2],
                   fillword[3], b[0], b[1], b[2], b[3]);
            wait_us(100);
        }
    }
    printf("BUS_DONE\r\n");
}

void phase_calibration(int ms) {
    printf("PCAL_START\r\n");
    Timer t;
    t.start();
    while (t.elapsed_time() < std::chrono::milliseconds(ms)) {
        for (int i = 0; i < N_IMU; i++) {
            if (!imu_ok[i]) continue;
            uint32_t now = clock_us.elapsed_time().count();
            uint16_t n, m;
            pipe_status(i, &n, &m);
            if (n > 0 && n <= sizeof(rx)) imu_transfer(i, OP_NOTIFICATION, nullptr, rx, n);
            if (m == 0 || m > sizeof(rx)) continue;
            imu_transfer(i, OP_MEASUREMENT, nullptr, rx, m);
            ImuSample s = {};
            parse_measurement(rx, m, s);
            if (s.n_new > 0) {
                printf("P,%d,%lu,%u,%lu,%u\r\n", i, (unsigned long)now, s.pc, (unsigned long)s.stf, s.n_new);
            }
        }
    }
    printf("PCAL_END\r\n");
}

int main() {
    spi_imu.format(8, 3);
    spi_imu.frequency(imu_spi_freq);
    spi_enc.format(16, 2);
    spi_enc.frequency(ENC_SPI_FREQ);
    clock_us.start();
    ThisThread::sleep_for(3000ms);  // 等 IMU 開機

    printf("\r\nMulti IMU sync test (r=init, g=start, x=stop)\r\n");
    init_imus();
    sample_thread.start(sample_loop);
    set_rate(rate_hz);

    FileHandle* console = mbed_file_handle(STDIN_FILENO);
    while (true) {
        char ch;
        if (console->readable() && console->read(&ch, 1) == 1) {
            if (ch == 'g') {
                streaming = true;
            } else if (ch == 'x') {
                streaming = false;
            } else if (ch >= '1' && ch <= '3') {
                const int freqs[] = {200000, 500000, 1000000};
                imu_spi_freq = freqs[ch - '1'];
                spi_imu.frequency(imu_spi_freq);
                printf("IMU SPI clock = %d Hz\r\n", imu_spi_freq);
            } else if (ch >= 'A' && ch <= 'E') {
                const int rates[] = {100, 200, 400, 500, 1000};
                set_rate(rates[ch - 'A']);
                printf("RATE,%d\r\n", rate_hz);
            } else if (ch == 'v') {
                verbose = !verbose;
                printf("VERBOSE,%d\r\n", verbose);
            } else if (ch == 'b') {
                streaming = false;
                ThisThread::sleep_for(50ms);
                bus_check();
            } else if (ch == 'p') {
                streaming = false;
                ThisThread::sleep_for(50ms);
                phase_calibration(2000);
            } else if (ch == 'r') {
                streaming = false;
                ThisThread::sleep_for(50ms);
                init_imus();
            }
        }
        Cycle* c = mail.try_get_for(5ms);
        if (c) {
            int k = 0;
            for (int i = 0; i < N_IMU; i++) k += imu_ok[i];
            printf("D,%lu,%lu,%u,%.2f,%u,%d", (unsigned long)c->cycle, (unsigned long)c->t_us, c->enc_off,
                   c->enc_deg, c->enc_err, k);
            for (int i = 0; i < N_IMU; i++) {
                if (!imu_ok[i]) continue;
                const ImuSample& s = c->imu[i];
                printf(",%d,%u,%u,%u,%u,%lu,%.2f,%.1f", i, s.off, s.n_new, s.bad, s.pc, (unsigned long)s.stf,
                       s.pitch, s.gy);
            }
            printf(",%u,%lu\r\n", c->busy, (unsigned long)dropped);
            mail.free(c);
        }
    }
}
