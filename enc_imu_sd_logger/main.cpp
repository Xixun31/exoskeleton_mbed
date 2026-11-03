// =====================================================================
// 2 顆 AS5145 (daisy chain 串接) + 1 顆 SPI IMU (Xsens MTi) 以 100Hz 同步讀取，存到 microSD
//
// 三個裝置各用一組 SPI，SD 卡寫入卡頓 (偶爾 100ms 以上) 不會影響感測器讀取：
//   取樣執行緒 (Realtime)：每 10ms 讀 encoder + IMU → 放進 Mail 佇列 (可緩衝 2.5 秒)
//   main 執行緒          ：從佇列取出寫 CSV，每秒 fflush + fsync 一次 (斷電最多掉約 1 秒)
//
// 取樣時機：IMU 的 DRDY 腳 (有新封包時拉高) 觸發 → 每個週期剛好一筆新 IMU 資料，
//           encoder 在 IMU 資料一出來就讀，兩者時間對齊。
//           DRDY 沒接時，改成每 1ms 輪詢 IMU pipe，有新封包才讀 (trig = 2，最多晚 1ms)；
//           IMU 沒回應時用 MCU Ticker 100Hz 只讀 encoder (trig = 0)。
//
// ---------------- 接線 (全部用 Morpho 公頭，板子正面) ----------------
//  [Encoder, SPI1]                          [IMU, SPI2]
//   SCK  PB_3  CN10-31                       SCK  PB_13 CN10-30
//   MISO PB_4  CN10-27  (接 encoder1 DO)      MISO PB_14 CN10-28
//   MOSI PB_5  CN10-29  (不用接)              MOSI PB_15 CN10-26
//   CS   PC_7  CN10-19                       CS   PB_1  CN10-24
//                                            DRDY PC_8  CN10-2
//  [microSD, SPI3]
//   SCK  PC_10 CN7-1    MISO PC_11 CN7-2    MOSI PC_12 CN7-3    CS PD_2 CN7-4
//  電源：3V3 CN7-16、5V CN7-18、GND CN7-8 / CN7-19 / CN7-20 / CN10-9 / CN10-20
//
//  AS5145 daisy chain：
//   CLK、CSn 兩顆並接；encoder1 DO → MCU MISO；encoder2 DO → encoder1 PROG；encoder2 PROG → GND
//   資料順序：encoder1 在前。每次讀 40 bit = 起始 1 bit + [enc1 18 bit] + dummy 1 bit + [enc2 18 bit] + 2 bit 不用
//   每顆 18 bit = D11..D0, OCF, COF, LIN, MagINC, MagDEC, Parity(偶同位)
//
// ---------------- 操作 ----------------
//  開機自動開始紀錄到 /fs/exoskeleton_project_record/enc_imu/runXXX.csv (不覆蓋舊檔)
//  藍色 B1 按鈕：短按 = 停止紀錄並安全卸載 SD (之後可拔卡)；再按一次 → 開新檔繼續紀錄
//              按住 3 秒 = 兩顆 encoder 歸零 (目前位置 = 0 deg，存在內部 flash，斷電 / 重燒都保留)
//  LD2 綠燈：慢閃 = 紀錄中、恆亮 = 已停止 (可拔卡)、快閃 = SD 錯誤
//  序列埠 115200：每秒印一行狀態 (enc_err = 通訊錯誤、mag_warn = 磁場警告)。指令 's' / 'S' 切換即時資料輸出 10Hz / 50Hz (python/imu/live_plot_enc_imu.py 畫圖)、'g' 開始、'x' 停止、'i' 重新初始化 IMU、
//  'z' encoder 歸零 (同 B1 按住 3 秒)
//
// ---------------- CSV 欄位 ----------------
//  cycle,t_s,trig,
//  enc1_raw,enc1_deg,enc1_stat,enc1_err,enc2_raw,enc2_deg,enc2_stat,enc2_err,
//  imu_new,imu_bad,imu_pc,imu_stf,roll,pitch,yaw,gx,gy,gz,ax,ay,az,busy_us,dropped
//   t_s      : 週期開始的 MCU 時間 (秒，µs 解析度)
//   trig     : 1 = IMU DRDY 觸發，2 = 輪詢到新 IMU 封包，0 = Ticker / IMU 逾時
//   encX_raw : 0~4095 (未歸零的原始值)；encX_deg 為相對零點的角度 -180~+180
//              (最後一筆通訊正確的值，通訊錯誤時沿用上一筆，磁場警告不影響)
//   encX_stat: 6 個狀態 bit (OCF COF LIN MagINC MagDEC Par)
//   encX_err : 0 = 正確；通訊錯誤 bit0 同位錯、bit1 OCF=0、bit2 COF=1、bit5 起始 bit 不是 1；
//              磁場警告 bit3 LIN=1、bit4 MagINC+MagDEC 同時為 1
//   imu_new  : 這個週期讀到幾個新封包 (DRDY 模式應恆為 1)；imu_bad = 格式 / checksum 錯誤次數
//   imu_pc   : PacketCounter；imu_stf = SampleTimeFine (0.1ms)
//   roll~yaw : deg；gx~gz : deg/s；ax~az : m/s^2 (IMU 沒有輸出加速度時，初始化會自動加進 IMU 的輸出設定)
//   busy_us  : 這個週期讀完所有裝置花的時間；dropped = 佇列滿而丟掉的週期數 (累計)
// =====================================================================
#include "mbed.h"
#include "SDBlockDevice.h"
#include "FATFileSystem.h"
#include <sys/stat.h>
#include <cerrno>

// ---------------- 接線設定 ----------------
SPI spi_enc(PB_5, PB_4, PB_3);  // mosi, miso, sclk (SPI1)
DigitalOut enc_cs(PC_7, 1);
const int ENC_SPI_FREQ = 1000000;  // AS5145 最高 1MHz (encoder_speed_test 實測 1MHz 0 錯誤)
const int N_ENC = 2;

SPI spi_imu(PB_15, PB_14, PB_13);  // mosi, miso, sclk (SPI2)
DigitalOut imu_cs(PB_1, 1);
InterruptIn imu_drdy(PC_8, PullDown);
const int IMU_SPI_FREQ = 1000000;

const int SD_SPI_FREQ = 4000000;  // 杜邦線太長、mount 失敗時改 1000000
SDBlockDevice sd(PC_12, PC_11, PC_10, PD_2, SD_SPI_FREQ);  // mosi, miso, sclk, cs (SPI3)
FATFileSystem fs("fs");

DigitalIn user_button(BUTTON1);  // PC13，按下 = 0
DigitalOut led(LED1);            // PA5 (LD2)

const auto PERIOD = 10ms;  // 100Hz
const float RAD2DEG = 57.2957795f;
const char* LOG_DIR = "/fs/exoskeleton_project_record/enc_imu";

// ---------------- MTi 協定 (同 multi_imu_sync) ----------------
const uint8_t OP_PROTINFO = 0x01, OP_CONFIGPROT = 0x02, OP_CONTROL = 0x03, OP_PIPESTATUS = 0x04,
              OP_NOTIFICATION = 0x05, OP_MEASUREMENT = 0x06;
const uint8_t MID_REQDID = 0x00, MID_DEVICEID = 0x01, MID_GOTOMEAS = 0x10, MID_GOTOMEAS_ACK = 0x11,
              MID_GOTOCONFIG = 0x30, MID_GOTOCONFIG_ACK = 0x31, MID_MTDATA2 = 0x36, MID_OUTPUTCONFIG = 0xC0,
              MID_OUTPUTCONFIG_ACK = 0xC1;

uint8_t fillword[4];
uint8_t rx[256];

void imu_transfer(uint8_t op, const uint8_t* tx, uint8_t* out, int len) {
    imu_cs = 0;
    fillword[0] = spi_imu.write(op);
    for (int k = 0; k < 3; k++) {
        fillword[k + 1] = spi_imu.write(k);
    }
    for (int k = 0; k < len; k++) {
        uint8_t b = spi_imu.write(tx ? tx[k] : 0x00);
        if (out) out[k] = b;
    }
    imu_cs = 1;
    wait_us(5);
}

void pipe_status(uint16_t* n, uint16_t* m) {
    uint8_t b[4];
    imu_transfer(OP_PIPESTATUS, nullptr, b, 4);
    *n = b[0] | (b[1] << 8);
    *m = b[2] | (b[3] << 8);
}

bool checksum_ok(const uint8_t* msg, int len) {
    uint8_t sum = 0xFF;
    for (int k = 0; k < len; k++) sum += msg[k];
    return sum == 0;
}

void send_xbus(uint8_t mid, const uint8_t* data = nullptr, uint8_t len = 0) {
    uint8_t msg[3 + 64];
    uint8_t sum = 0xFF + mid + len;
    msg[0] = mid;
    msg[1] = len;
    for (int k = 0; k < len; k++) {
        msg[2 + k] = data[k];
        sum += data[k];
    }
    msg[2 + len] = (uint8_t)(-sum);
    imu_transfer(OP_CONTROL, msg, nullptr, 3 + len);
}

int wait_notification(uint8_t want, uint8_t* payload, int timeout_ms);

// 讀 IMU 的輸出設定 (每項 4 bytes：XDI 2 + 頻率 2)，沒有加速度就加進去 (會存進 IMU，MT Manager 可改回)
void ensure_imu_acc() {
    uint8_t cfg[64];
    send_xbus(MID_OUTPUTCONFIG);
    int len = wait_notification(MID_OUTPUTCONFIG_ACK, cfg, 500);
    if (len < 0 || len % 4 != 0) {
        printf("IMU output config: 讀取失敗 (%d)\r\n", len);
        return;
    }
    bool has_acc = false;
    uint16_t freq = 100, low = 0;
    printf("IMU output config:");
    for (int k = 0; k < len; k += 4) {
        uint16_t xdi = (cfg[k] << 8) | cfg[k + 1], f = (cfg[k + 2] << 8) | cfg[k + 3];
        printf(" %04X@%u", xdi, f);
        if ((xdi & 0xFFF0) == 0x4020) has_acc = true;
        if ((xdi & 0xFFF0) == 0x2030 || (xdi & 0xFFF0) == 0x8020) {
            freq = f;
            low = xdi & 0x000F;  // 座標系 / 精度跟姿態角、陀螺儀相同
        }
    }
    printf("\r\n");
    if (has_acc || len + 4 > (int)sizeof(cfg)) return;
    uint16_t acc = 0x4020 | low;
    cfg[len] = acc >> 8;
    cfg[len + 1] = acc & 0xFF;
    cfg[len + 2] = freq >> 8;
    cfg[len + 3] = freq & 0xFF;
    send_xbus(MID_OUTPUTCONFIG, cfg, len + 4);
    int r = wait_notification(MID_OUTPUTCONFIG_ACK, nullptr, 2000);
    printf("IMU output config: 加入加速度 %04X@%u -> %s\r\n", acc, freq, r >= 0 ? "OK" : "FAIL");
}

int wait_notification(uint8_t want, uint8_t* payload, int timeout_ms) {
    Timer t;
    t.start();
    while (t.elapsed_time() < std::chrono::milliseconds(timeout_ms)) {
        uint16_t n, m;
        pipe_status(&n, &m);
        if (n > 0 && n <= sizeof(rx)) {
            imu_transfer(OP_NOTIFICATION, nullptr, rx, n);
            if (rx[0] == want && checksum_ok(rx, n)) {
                if (payload) memcpy(payload, &rx[2], rx[1]);
                return rx[1];
            }
        } else if (m > 0 && m <= sizeof(rx)) {
            imu_transfer(OP_MEASUREMENT, nullptr, rx, m);
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

bool imu_ok = false;

void init_imu() {
    uint8_t info[2];
    imu_transfer(OP_PROTINFO, nullptr, info, 2);
    bool prot = fillword[0] == 0xFA && fillword[1] == 0xFF && fillword[2] == 0xFF && fillword[3] == 0xFF;
    uint32_t did = 0;
    bool meas = false;
    if (prot) {
        // 0x08：DRDY = push-pull、平時 Low，measurement pipe 有資料時拉高
        const uint8_t cfg = 0x08;
        imu_transfer(OP_CONFIGPROT, &cfg, nullptr, 1);
        send_xbus(MID_GOTOCONFIG);
        wait_notification(MID_GOTOCONFIG_ACK, nullptr, 500);
        uint8_t d[8];
        send_xbus(MID_REQDID);
        if (wait_notification(MID_DEVICEID, d, 500) == 4) {
            did = ((uint32_t)d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3];
        }
        if (did) ensure_imu_acc();
        send_xbus(MID_GOTOMEAS);
        meas = wait_notification(MID_GOTOMEAS_ACK, nullptr, 500) >= 0;
    }
    imu_ok = prot && did != 0;
    printf("IMU init: fw=%02X%02X%02X%02X did=%08lX meas=%d ok=%d\r\n", fillword[0], fillword[1], fillword[2],
           fillword[3], (unsigned long)did, meas, imu_ok);
    if (!prot) {
        // 診斷：MISO 加內部上拉再讀一次。FFFFFFFF = MISO 沒有被驅動 (線沒接通 / IMU 沒開機 / 非 SPI 模式)；
        // 仍為 00000000 = MISO 被拉 Low (接錯腳或短路)
        pin_mode(PB_14, PullUp);
        imu_transfer(OP_PROTINFO, nullptr, info, 2);
        pin_mode(PB_14, PullNone);
        printf("IMU diag (MISO pull-up): fw=%02X%02X%02X%02X\r\n", fillword[0], fillword[1], fillword[2], fillword[3]);
    }
}

// ---------------- 每週期的資料 ----------------
struct ImuData {
    uint8_t n_new, bad;
    uint16_t pc;
    uint32_t stf;
    float euler[3], gyr[3], acc[3];
};
struct EncData {
    uint16_t raw;
    uint8_t stat, err;
    float deg;
};
struct Sample {
    uint32_t cycle;
    uint64_t t_us;
    uint8_t trig;
    EncData enc[N_ENC];
    ImuData imu;
    uint16_t busy;
    uint32_t dropped;
};
Mail<Sample, 256> mail;
volatile uint32_t dropped = 0;
Timer clock_us;

uint64_t now_us() { return clock_us.elapsed_time().count(); }

// 解析一段 measurement pipe 資料 (可能含多個 MTData2 訊息)，更新最新值
void parse_measurement(const uint8_t* buf, int len, ImuData& s) {
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
                case 0x2030: if (size == 12) for (int k = 0; k < 3; k++) s.euler[k] = be_float(c + 4 * k); break;
                case 0x4020: if (size == 12) for (int k = 0; k < 3; k++) s.acc[k] = be_float(c + 4 * k); break;
                case 0x8020: if (size == 12) for (int k = 0; k < 3; k++) s.gyr[k] = be_float(c + 4 * k) * RAD2DEG; break;
            }
            i += 3 + size;
        }
        off += mlen;
    }
}

void read_imu(ImuData& s) {
    if (!imu_ok) return;
    for (int guard = 0; guard < 4; guard++) {  // 把積壓的封包都讀完 (讀完 DRDY 才會回到 Low)
        uint16_t n, m;
        pipe_status(&n, &m);
        if (n > 0 && n <= sizeof(rx)) {
            imu_transfer(OP_NOTIFICATION, nullptr, rx, n);
        }
        if (m == 0) break;
        if (m > sizeof(rx)) {
            s.bad++;
            break;
        }
        imu_transfer(OP_MEASUREMENT, nullptr, rx, m);
        parse_measurement(rx, m, s);
    }
}

// 一次讀 40 bit，回傳 MSB 對齊在 bit39
uint64_t read_encoder_bits() {
    char tx[5] = {}, b[5];
    enc_cs = 0;
    wait_us(1);
    spi_enc.write(tx, 5, b, 5);
    enc_cs = 1;
    uint64_t v = 0;
    for (int k = 0; k < 5; k++) v = (v << 8) | (uint8_t)b[k];
    return v;
}

// ---------------- encoder 歸零 ----------------
// 零點存在內部 flash 最後一個 sector (sector 7，128KB，與程式隔得最遠)，斷電 / 重新燒錄都會保留
const uint32_t ZERO_ADDR = 0x08060000;
const uint32_t ZERO_MAGIC = 0x5A45524F;  // "ZERO"
struct ZeroStore {
    uint32_t magic;
    uint16_t zero[N_ENC];
    uint32_t check;
};
volatile uint16_t enc_zero[N_ENC] = {};
bool zero_valid = false;

uint32_t zero_check(const ZeroStore& z) { return z.magic ^ ((uint32_t)z.zero[0] << 16 | z.zero[1]) ^ 0xA5A5A5A5; }

void load_zero() {
    ZeroStore z;
    memcpy(&z, (const void*)ZERO_ADDR, sizeof(z));
    zero_valid = z.magic == ZERO_MAGIC && z.check == zero_check(z) && z.zero[0] < 4096 && z.zero[1] < 4096;
    for (int i = 0; i < N_ENC; i++) enc_zero[i] = zero_valid ? z.zero[i] : 0;
    printf("ENC zero: %s enc1=%u enc2=%u\r\n", zero_valid ? "loaded" : "not set (用 raw)", enc_zero[0], enc_zero[1]);
}

// 抹除 sector 時 CPU 會停住約 1~2 秒 (取樣也會停)，只在歸零時用
bool save_zero(const uint16_t* zero) {
    ZeroStore z = {ZERO_MAGIC, {zero[0], zero[1]}, 0};
    z.check = zero_check(z);
    FlashIAP flash;
    if (flash.init() != 0) return false;
    bool ok = flash.erase(ZERO_ADDR, flash.get_sector_size(ZERO_ADDR)) == 0 &&
              flash.program(&z, ZERO_ADDR, sizeof(z)) == 0;
    flash.deinit();
    return ok;
}

// raw 相對零點的角度，範圍 -180 ~ +180 deg
float zeroed_deg(uint16_t raw, int i) {
    int d = ((int)raw - (int)enc_zero[i]) & 0x0FFF;
    if (d >= 2048) d -= 4096;
    return d * (360.0f / 4096.0f);
}

// 第 i 顆 (0 = 最靠近 MCU 的 encoder1) 的 18 bit：起始 1 bit 之後，每顆 18 bit + 1 dummy bit
void parse_encoder(uint64_t v, int i, EncData& e) {
    uint32_t w = (v >> (39 - 1 - 19 * i - 17)) & 0x3FFFF;
    e.raw = w >> 6;
    e.stat = w & 0x3F;
    bool ocf = w & 0x20, cof = w & 0x10, lin = w & 0x08, inc = w & 0x04, dec = w & 0x02;
    e.err = 0;
    if (__builtin_parity(w)) e.err |= 0x01;
    if (!ocf) e.err |= 0x02;
    if (cof) e.err |= 0x04;
    if (lin) e.err |= 0x08;
    if (inc && dec) e.err |= 0x10;
    if (!((v >> 39) & 1)) e.err |= 0x20;
    // 通訊錯誤 (同位 / OCF / COF / 起始 bit) 時沿用上一筆；磁場警告 (LIN、MagINC+MagDEC) 仍採用讀值，只記在 err
    if ((e.err & 0x27) == 0) e.deg = zeroed_deg(e.raw, i);
}

// ---------------- 取樣執行緒 ----------------
Thread sample_thread(osPriorityRealtime, 4096);
Ticker sample_ticker;
EventFlags sample_flag;
const uint32_t FLAG_DRDY = 1, FLAG_TICK = 2;
volatile bool use_drdy = false;

void on_drdy() { sample_flag.set(FLAG_DRDY); }
void on_tick() { sample_flag.set(FLAG_TICK); }

const auto POLL_PERIOD = 1ms;       // 沒有 DRDY 時輪詢 IMU pipe 的間隔
const auto IMU_STALL = 20ms;        // 輪詢模式下超過這個時間沒有新封包，照樣讀一次 encoder
volatile bool use_poll = false;

// 沒有 DRDY 時：IMU 正常就每 1ms 輪詢 pipe (有新封包才跑一個週期，等同軟體 DRDY)；
// IMU 沒初始化就用 100Hz Ticker 只讀 encoder
void use_ticker() {
    use_drdy = false;
    imu_drdy.rise(nullptr);
    use_poll = imu_ok;
    sample_ticker.attach(callback(on_tick), use_poll ? POLL_PERIOD : PERIOD);
}

void sample_loop() {
    uint32_t cycle = 0;
    Sample last = {};
    int timeouts = 0;
    uint64_t last_cycle_us = 0;
    if (use_drdy) {
        ImuData tmp = {};
        read_imu(tmp);  // 先清空 pipe，讓 DRDY 回到 Low，之後才有上升緣
    }
    while (true) {
        uint8_t trig;
        if (use_drdy) {
            uint32_t f = sample_flag.wait_any_for(FLAG_DRDY, 20ms);
            if (f & osFlagsError) {
                // 20ms 沒有 DRDY：照樣讀一次；連續 5 次就改用輪詢
                trig = 0;
                if (++timeouts >= 5) {
                    use_ticker();
                    printf("WARN: no IMU DRDY, switch to 1ms IMU polling\r\n");
                }
            } else {
                trig = 1;
                timeouts = 0;
            }
        } else if (use_poll) {
            sample_flag.wait_any(FLAG_TICK);
            uint16_t n, m;
            pipe_status(&n, &m);
            if (m == 0) {
                if (n > 0 && n <= sizeof(rx)) imu_transfer(OP_NOTIFICATION, nullptr, rx, n);
                if (now_us() - last_cycle_us < (uint64_t)std::chrono::microseconds(IMU_STALL).count()) continue;
                trig = 0;  // IMU 停住了：照樣讀 encoder
            } else {
                trig = 2;
            }
        } else {
            sample_flag.wait_any(FLAG_TICK);
            trig = 0;
        }

        Sample s = last;
        uint64_t t0 = now_us();
        last_cycle_us = t0;
        s.cycle = cycle++;
        s.t_us = t0;
        s.trig = trig;
        uint64_t v = read_encoder_bits();
        for (int i = 0; i < N_ENC; i++) parse_encoder(v, i, s.enc[i]);
        s.imu.n_new = 0;
        s.imu.bad = 0;
        read_imu(s.imu);
        s.busy = (uint16_t)(now_us() - t0);
        s.dropped = dropped;
        last = s;

        Sample* p = mail.try_alloc();
        if (p) {
            *p = s;
            mail.put(p);
        } else {
            dropped++;
        }
    }
}

// ---------------- SD 紀錄 ----------------
FILE* log_file = nullptr;
bool sd_mounted = false;
bool sd_error = false;
char log_path[80] = {};
static char sd_stdio_buffer[8192];
uint32_t lines_written = 0;

bool open_log_file() {
    if (!sd_mounted) {
        int err = sd.init();
        if (err != 0) {
            printf("SD init failed: %d\r\n", err);
            return false;
        }
        err = fs.mount(&sd);
        if (err != 0) {
            printf("SD mount failed: %d (請確認為 FAT32)\r\n", err);
            sd.deinit();
            return false;
        }
        sd_mounted = true;
    }
    mkdir("/fs/exoskeleton_project_record", 0777);  // 已存在時失敗可忽略
    mkdir(LOG_DIR, 0777);

    struct stat st;
    bool found = false;
    for (int i = 1; i <= 999; i++) {
        snprintf(log_path, sizeof(log_path), "%s/run%03d.csv", LOG_DIR, i);
        if (stat(log_path, &st) != 0) {
            found = true;
            break;
        }
    }
    if (!found) {
        printf("找不到可用檔名 run001~run999.csv\r\n");
        return false;
    }
    log_file = fopen(log_path, "w");
    if (!log_file) {
        printf("無法建立 %s (errno=%d)\r\n", log_path, errno);
        return false;
    }
    setvbuf(log_file, sd_stdio_buffer, _IOFBF, sizeof(sd_stdio_buffer));
    fprintf(log_file,
            "cycle,t_s,trig,enc1_raw,enc1_deg,enc1_stat,enc1_err,enc2_raw,enc2_deg,enc2_stat,enc2_err,"
            "imu_new,imu_bad,imu_pc,imu_stf,roll,pitch,yaw,gx,gy,gz,ax,ay,az,busy_us,dropped\n");
    fflush(log_file);
    lines_written = 0;
    printf("REC start: %s\r\n", log_path);
    return true;
}

bool start_recording() {
    if (log_file) return true;
    sd_error = !open_log_file();
    return !sd_error;
}

void stop_recording() {
    if (log_file) {
        fflush(log_file);
        fclose(log_file);
        log_file = nullptr;
        printf("REC stop: %s, %lu lines\r\n", log_path, (unsigned long)lines_written);
    }
    if (sd_mounted) {
        fs.unmount();
        sd.deinit();
        sd_mounted = false;
        printf("SD unmounted，可以拔卡\r\n");
    }
}

// fflush 只把資料交給檔案系統；fsync 才會更新 FAT 與檔案大小，斷電後檔案才讀得到
bool sync_recording() {
    if (!log_file) return true;
    if (fflush(log_file) != 0 || fsync(fileno(log_file)) != 0 || ferror(log_file)) {
        printf("SD write error (errno=%d)，停止紀錄\r\n", errno);
        fclose(log_file);
        log_file = nullptr;
        fs.unmount();
        sd.deinit();
        sd_mounted = false;
        sd_error = true;
        return false;
    }
    return true;
}

void write_sample(FILE* f, const Sample& s) {
    fprintf(f, "%lu,%lu.%06lu,%u", (unsigned long)s.cycle, (unsigned long)(s.t_us / 1000000),
            (unsigned long)(s.t_us % 1000000), s.trig);
    for (int i = 0; i < N_ENC; i++) {
        fprintf(f, ",%u,%.2f,%02X,%u", s.enc[i].raw, s.enc[i].deg, s.enc[i].stat, s.enc[i].err);
    }
    const ImuData& m = s.imu;
    fprintf(f, ",%u,%u,%u,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%u,%lu\n", m.n_new, m.bad, m.pc,
            (unsigned long)m.stf, m.euler[0], m.euler[1], m.euler[2], m.gyr[0], m.gyr[1], m.gyr[2], m.acc[0],
            m.acc[1], m.acc[2], s.busy, (unsigned long)s.dropped);
}

void print_encoder_check() {
    uint64_t v = read_encoder_bits();
    printf("ENC raw40=%02X%08lX\r\n", (unsigned)(v >> 32), (unsigned long)(v & 0xFFFFFFFF));
    for (int i = 0; i < N_ENC; i++) {
        EncData e = {};
        parse_encoder(v, i, e);
        printf("  enc%d raw=%4u deg=%7.2f stat=%02X err=%02X\r\n", i + 1, e.raw, zeroed_deg(e.raw, i), e.stat, e.err);
    }
}

int main() {
    spi_enc.format(8, 2);  // AS5145：CLK 平時 High，下降緣取樣 (同 encoder/main.cpp 的 Mode 2)
    spi_enc.frequency(ENC_SPI_FREQ);
    spi_imu.format(8, 3);
    spi_imu.frequency(IMU_SPI_FREQ);
    user_button.mode(PullUp);
    clock_us.start();

    printf("\r\n==== 2x AS5145 + IMU -> microSD @100Hz ====\r\n");
    ThisThread::sleep_for(3000ms);  // 等 IMU 開機

    init_imu();
    load_zero();
    print_encoder_check();

    if (imu_ok) {
        use_drdy = true;
        imu_drdy.rise(callback(on_drdy));
    } else {
        printf("WARN: IMU 未初始化，使用 100Hz ticker，只記錄 encoder\r\n");
        use_ticker();
    }
    sample_thread.start(sample_loop);
    start_recording();  // 放在取樣開始之後：SD 沒插或初始化很慢時，感測器照常讀取

    FileHandle* console = mbed_file_handle(STDIN_FILENO);
    int live_div = 0;  // 即時輸出：0 = 關、10 = 每 10 個週期一筆 (10Hz)、2 = 50Hz
    int zero_n = 0;                 // > 0：歸零中，還要收集幾筆
    int32_t zero_sum[N_ENC] = {};   // 相對第一筆的差值總和 (處理 4095→0 跨越)
    uint16_t zero_ref[N_ENC] = {};
    int zero_got = 0;
    bool long_done = false;
    auto request_zero = [&]() {
        zero_n = 50;
        zero_got = 0;
        printf("ZERO: 收集 50 筆中，請保持不動...\r\n");
    };
    bool button_prev = false;
    uint64_t button_t = 0, last_status = now_us(), last_led = 0;
    uint32_t n_sec = 0, enc_err[N_ENC] = {}, enc_warn[N_ENC] = {}, imu_miss = 0, imu_bad = 0, trig_drdy = 0;
    uint16_t busy_max = 0;

    while (true) {
        Sample* s = mail.try_get_for(5ms);
        if (s) {
            if (log_file) {
                write_sample(log_file, *s);
                lines_written++;
            }
            if (live_div && s->cycle % live_div == 0) {
                printf("D,%lu,%.2f,%u,%.2f,%u,%u,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f\r\n",
                       (unsigned long)s->cycle, s->enc[0].deg, s->enc[0].err, s->enc[1].deg, s->enc[1].err,
                       s->imu.n_new, s->imu.euler[0], s->imu.euler[1], s->imu.euler[2], s->imu.gyr[0], s->imu.gyr[1],
                       s->imu.gyr[2], s->imu.acc[0], s->imu.acc[1], s->imu.acc[2]);
            }
            if (zero_n > 0 && !(s->enc[0].err & 0x27) && !(s->enc[1].err & 0x27)) {
                for (int i = 0; i < N_ENC; i++) {
                    if (zero_got == 0) {
                        zero_ref[i] = s->enc[i].raw;
                        zero_sum[i] = 0;
                    }
                    int d = ((int)s->enc[i].raw - zero_ref[i]) & 0x0FFF;
                    if (d >= 2048) d -= 4096;
                    zero_sum[i] += d;
                }
                zero_got++;
                if (--zero_n == 0) {
                    uint16_t z[N_ENC];
                    for (int i = 0; i < N_ENC; i++) {
                        int avg = (int)lroundf((float)zero_sum[i] / zero_got);
                        z[i] = (uint16_t)((zero_ref[i] + avg) & 0x0FFF);
                        enc_zero[i] = z[i];
                    }
                    zero_valid = true;
                    // 抹除 flash 時 CPU 會停住；先停掉 1kHz Ticker / DRDY 中斷，不然 RTOS 的 ISR 佇列會塞爆當機
                    sample_ticker.detach();
                    imu_drdy.rise(nullptr);
                    bool ok = save_zero(z);
                    if (use_drdy) imu_drdy.rise(callback(on_drdy));
                    else sample_ticker.attach(callback(on_tick), use_poll ? POLL_PERIOD : PERIOD);
                    printf("ZERO: enc1=%u enc2=%u saved=%d (取樣暫停約 1~2 秒)\r\n", z[0], z[1], ok);
                }
            }
            n_sec++;
            for (int i = 0; i < N_ENC; i++) {
                enc_err[i] += (s->enc[i].err & 0x27) != 0;
                enc_warn[i] += (s->enc[i].err & 0x18) != 0;
            }
            imu_miss += imu_ok && s->imu.n_new == 0;
            imu_bad += s->imu.bad;
            trig_drdy += s->trig != 0;
            if (s->busy > busy_max) busy_max = s->busy;
            mail.free(s);
        }

        uint64_t now = now_us();

        // 每秒：寫回 SD + 印狀態
        if (now - last_status >= 1000000) {
            last_status = now;
            sync_recording();
            printf("STAT rec=%d lines=%lu rate=%lu imu_trig=%lu enc_err=%lu/%lu mag_warn=%lu/%lu imu_miss=%lu imu_bad=%lu busy_max=%uus "
                   "dropped=%lu\r\n",
                   log_file != nullptr, (unsigned long)lines_written, (unsigned long)n_sec,
                   (unsigned long)trig_drdy, (unsigned long)enc_err[0], (unsigned long)enc_err[1], (unsigned long)enc_warn[0],
                   (unsigned long)enc_warn[1],
                   (unsigned long)imu_miss, (unsigned long)imu_bad, busy_max, (unsigned long)dropped);
            n_sec = trig_drdy = imu_miss = imu_bad = busy_max = 0;
            for (int i = 0; i < N_ENC; i++) enc_err[i] = enc_warn[i] = 0;
        }

        // B1：50ms 消抖，按下切換 停止 / 開新檔
        bool pressed = user_button.read() == 0;
        // B1：短按 (放開時) 切換 停止 / 開新檔；按住 3 秒 = encoder 歸零
        if (pressed != button_prev && now - button_t > 50000) {
            button_prev = pressed;
            button_t = now;
            if (!pressed && !long_done) {
                if (log_file) stop_recording();
                else start_recording();
            }
            long_done = false;
        }
        if (pressed && button_prev && !long_done && now - button_t > 3000000) {
            long_done = true;
            request_zero();
        }

        if (console->readable()) {
            char ch;
            if (console->read(&ch, 1) == 1) {
                if (ch == 's' || ch == 'S') {
                    int div = ch == 's' ? 10 : 2;
                    live_div = live_div == div ? 0 : div;
                }
                else if (ch == 'g') start_recording();
                else if (ch == 'x') stop_recording();
                else if (ch == 'z') request_zero();
                else if (ch == 'i' && !imu_ok) {
                    // 重新初始化 IMU (換 IMU 後不用重開機)；成功就改用 DRDY 觸發
                    init_imu();
                    if (imu_ok) {
                        sample_ticker.detach();
                        use_poll = false;
                        use_drdy = true;
                        imu_drdy.rise(callback(on_drdy));
                        sample_flag.set(FLAG_TICK);  // 叫醒正在等 Ticker 的取樣執行緒；這次讀取會清空 pipe，DRDY 回到 Low
                    }
                }
            }
        }

        // LED：紀錄中慢閃、停止恆亮、SD 錯誤快閃
        if (log_file) {
            if (now - last_led >= 500000) {
                led = !led;
                last_led = now;
            }
        } else if (sd_error) {
            if (now - last_led >= 100000) {
                led = !led;
                last_led = now;
            }
        } else {
            led = 1;
        }
    }
}
