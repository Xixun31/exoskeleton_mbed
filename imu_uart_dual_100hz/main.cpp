// =====================================================================
// 兩顆 UART IMU (Xsens MTi, MTData2) 以 100Hz 讀取測試
// 腳位與之後要併入 enc_imu_sd_logger 的相同，不會跟那邊的 SPI1/2/3 衝突：
//
//   IMU_A : USART3  TX PB_10 (Arduino D6) -> IMU RX
//                   RX PC_5  (CN10)       <- IMU TX
//   IMU_B : UART4   TX PA_0  (Arduino A0) -> IMU RX
//                   RX PA_1  (Arduino A1) <- IMU TX
//   兩顆 IMU 的 GND 都要接到板子 GND；IMU UART 必須是 3.3V TTL 準位
//
// IMU 的輸出 (Euler 0x2030 / Acc 0x4020 / Gyro 0x8020，建議再加 PacketCounter
// 0x1020 用來算掉包) 與鮑率要先用 MT Manager 設好，這裡只負責接收。
//
// 序列埠 (ST-Link USB) 921600 baud：screen /dev/ttyACM0 921600
//   預設 : 每秒印一次統計
//   'g'  : 切換 100Hz CSV 串流 (每 10ms 一行)
//   'c'  : 統計歸零
//   'i'  : 對兩顆 IMU 送 ReqDID，有回 DeviceID 代表 IMU 有在聽 UART
//   'm'  : 送 GoToMeasurement (開機時也會自動送一次)
//   'k'  : 送 GoToConfig (停止輸出)
//   's'  : 掃描鮑率：每種鮑率送 ReqDID，看哪個有回應 (掃完回到 IMU_BAUD[])
//
// CSV 格式：D,cycle,t_ms, [每顆 IMU] new,pc,age_ms,roll,pitch,yaw,ax,ay,az,gx,gy,gz
//   new    : 這個 10ms 週期收到幾個新封包 (正常為 1；偶爾 0/2 是兩邊時脈不同步造成的)
//   pc     : PacketCounter (IMU 沒輸出就是 -1)
//   age_ms : 最新封包收到到現在經過多久
// =====================================================================
#include "mbed.h"
#include <cmath>

// ---------------- 設定 ----------------
// 要跟各 IMU 設定的一樣 (用 's' 掃描可以查)。目前一包約 69 byte，115200 在 100Hz 時約 60% 負載
const int IMU_BAUD[2] = {921600, 115200};  // IMU_A, IMU_B
const int LOOP_HZ = 100;
const float RAD2DEG = 57.2957795f;

// ---------------- MTi UART 接收器 ----------------
struct ImuSample {
    float euler[3];  // deg
    float accel[3];  // m/s^2
    float gyro[3];   // rad/s
    int32_t packet_counter;  // -1 = 封包裡沒有 PacketCounter
    uint32_t t_us;           // 收到這個封包時的 MCU 時間
};

class MtiUart {
public:
    // 以下計數只在 ISR 裡累加，main 讀取時用 critical section 取一致的快照
    volatile uint32_t rx_bytes = 0;
    volatile uint32_t pkt_ok = 0;
    volatile uint32_t pkt_bad_cksum = 0;
    volatile uint32_t pkt_bad_len = 0;

    MtiUart(PinName tx, PinName rx, int baud) : serial(tx, rx, baud) {
        memset(&latest, 0, sizeof(latest));
        latest.packet_counter = -1;
    }

    volatile uint32_t msg_other = 0;  // 非 MTData2 的訊息數 (ACK、DeviceID、Error...)
    volatile uint8_t last_other_mid = 0xFF;
    volatile uint32_t device_id = 0;

    void set_baud(int b) { serial.baud(b); state = S_PRE; }

    void start() { serial.attach(callback(this, &MtiUart::rx_isr), SerialBase::RxIrq); }

    // 送一個 MTi 指令 (無資料)：FA FF MID 00 CKS
    void send_cmd(uint8_t m) {
        uint8_t msg[5] = {0xFA, 0xFF, m, 0x00, (uint8_t)(0x100 - ((0xFF + m) & 0xFF))};
        serial.write(msg, 5);
    }

    // 複製最新一筆資料，回傳從上次呼叫到現在收到幾個新封包
    int fetch(ImuSample& out) {
        core_util_critical_section_enter();
        out = latest;
        int n = (int)(pkt_ok - fetched);
        fetched = pkt_ok;
        core_util_critical_section_exit();
        return n;
    }

    void reset_stats() {
        core_util_critical_section_enter();
        rx_bytes = pkt_ok = pkt_bad_cksum = pkt_bad_len = 0;
        fetched = 0;
        core_util_critical_section_exit();
    }

private:
    UnbufferedSerial serial;
    ImuSample latest;
    uint32_t fetched = 0;

    // 封包格式：FA FF MID LEN payload[LEN] CHECKSUM
    // checksum：FF + MID + LEN + payload + CHECKSUM 加總的低 8 bit = 0
    enum { S_PRE, S_BID, S_MID, S_LEN, S_DATA, S_CKS } state = S_PRE;
    uint8_t mid = 0, len = 0, idx = 0, sum = 0;
    uint8_t buf[256];

    static float be_float(const uint8_t* d) {
        uint32_t t = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3];
        float f;
        memcpy(&f, &t, 4);
        return f;
    }

    void parse_mtdata2(const uint8_t* d, int n) {
        int i = 0;
        while (i + 3 <= n) {
            uint16_t xdi = (d[i] << 8) | d[i + 1];
            uint8_t size = d[i + 2];
            if (i + 3 + size > n) break;  // 宣告長度超出封包，丟棄剩下的
            const uint8_t* c = &d[i + 3];
            uint16_t id = xdi & 0xFFF0;   // 低 4 bit 是座標系 / 精度，float32 時 size 一定是 12
            if (id == 0x2030 && size == 12) {
                for (int k = 0; k < 3; k++) latest.euler[k] = be_float(c + 4 * k);
            } else if (id == 0x4020 && size == 12) {
                for (int k = 0; k < 3; k++) latest.accel[k] = be_float(c + 4 * k);
            } else if (id == 0x8020 && size == 12) {
                for (int k = 0; k < 3; k++) latest.gyro[k] = be_float(c + 4 * k);
            } else if (xdi == 0x1020 && size == 2) {
                latest.packet_counter = (c[0] << 8) | c[1];
            }
            i += 3 + size;
        }
    }

    void feed(uint8_t b) {
        switch (state) {
            case S_PRE:
                if (b == 0xFA) state = S_BID;
                break;
            case S_BID:
                if (b == 0xFF) { sum = b; state = S_MID; }
                else state = (b == 0xFA) ? S_BID : S_PRE;
                break;
            case S_MID:
                mid = b; sum += b; state = S_LEN;
                break;
            case S_LEN:
                len = b; sum += b; idx = 0;
                if (len == 0xFF) { pkt_bad_len++; state = S_PRE; }  // extended length 不支援
                else state = (len == 0) ? S_CKS : S_DATA;
                break;
            case S_DATA:
                buf[idx++] = b; sum += b;
                if (idx >= len) state = S_CKS;
                break;
            case S_CKS:
                sum += b;
                if (sum != 0) {
                    pkt_bad_cksum++;
                } else if (mid == 0x36) {
                    parse_mtdata2(buf, len);
                    latest.t_us = us_ticker_read();
                    pkt_ok++;
                } else {
                    if (mid == 0x01 && len == 4)
                        device_id = ((uint32_t)buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
                    last_other_mid = mid;
                    msg_other++;
                }
                state = S_PRE;
                break;
        }
    }

    void rx_isr() {
        char c;
        while (serial.readable()) {
            if (serial.read(&c, 1) != 1) break;
            rx_bytes++;
            feed((uint8_t)c);
        }
    }
};

// ---------------- 全域 ----------------
const int N_IMU = 2;
MtiUart imu[N_IMU] = {{PB_10, PC_5, IMU_BAUD[0]}, {PA_0, PA_1, IMU_BAUD[1]}};
const char* IMU_NAME[N_IMU] = {"IMU_A(USART3 RX=PC5)", "IMU_B(UART4  RX=PA1)"};
DigitalOut led(LED1);

// 每秒統計
struct Stats {
    uint32_t cycles_new0 = 0;  // 這個週期沒收到新封包
    uint32_t cycles_new2 = 0;  // 這個週期收到 >=2 個 (積壓)
    uint32_t lost = 0;         // 依 PacketCounter 推算 IMU 有送但沒收到/checksum 壞掉的封包
    int32_t last_pc = -1;
    uint32_t max_age_us = 0;
};
Stats st[N_IMU];

void reset_all() {
    for (int i = 0; i < N_IMU; i++) {
        imu[i].reset_stats();
        st[i] = Stats();
    }
}

int main() {
    printf("\r\n=== Dual UART IMU 100Hz test ===\r\n");
    printf("IMU_A baud %d, IMU_B baud %d, loop %d Hz. 'g' = CSV 串流, 'c' = 統計歸零, 's' = 掃描鮑率\r\n",
           IMU_BAUD[0], IMU_BAUD[1], LOOP_HZ);

    FileHandle* in = mbed_file_handle(STDIN_FILENO);
    in->set_blocking(false);

    for (int i = 0; i < N_IMU; i++) imu[i].start();
    // 開機先問 DeviceID，再要求進入量測模式 (IMU 若停在 Config 模式就不會輸出)
    ThisThread::sleep_for(500ms);
    for (int i = 0; i < N_IMU; i++) imu[i].send_cmd(0x00);
    ThisThread::sleep_for(100ms);
    for (int i = 0; i < N_IMU; i++) imu[i].send_cmd(0x10);

    bool stream = false;
    uint32_t cycle = 0;
    uint32_t prev_ok[N_IMU] = {0, 0};
    uint32_t prev_bytes[N_IMU] = {0, 0};
    uint32_t t0 = us_ticker_read();
    auto next = Kernel::Clock::now();
    const auto period = std::chrono::milliseconds(1000 / LOOP_HZ);  // Kernel::Clock 單位是 ms

    while (true) {
        next += period;
        ThisThread::sleep_until(next);
        cycle++;
        uint32_t now = us_ticker_read();

        // 指令
        char ch;
        while (in->read(&ch, 1) == 1) {
            if (ch == 'g') {
                stream = !stream;
                if (stream)
                    printf("D,cycle,t_ms,"
                           "A_new,A_pc,A_age,A_roll,A_pitch,A_yaw,A_ax,A_ay,A_az,A_gx,A_gy,A_gz,"
                           "B_new,B_pc,B_age,B_roll,B_pitch,B_yaw,B_ax,B_ay,B_az,B_gx,B_gy,B_gz\r\n");
            } else if (ch == 'i' || ch == 'm' || ch == 'k') {
                uint8_t m = (ch == 'i') ? 0x00 : (ch == 'm') ? 0x10 : 0x30;
                for (int i = 0; i < N_IMU; i++) imu[i].send_cmd(m);
                printf("-- 送出 MID 0x%02X 給兩顆 IMU --\r\n", m);
            } else if (ch == 's') {
                const int bauds[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 2000000};
                printf("-- 掃描鮑率 --\r\n");
                for (int b : bauds) {
                    for (int i = 0; i < N_IMU; i++) imu[i].set_baud(b);
                    ThisThread::sleep_for(50ms);
                    uint32_t b0[N_IMU], m0[N_IMU];
                    for (int i = 0; i < N_IMU; i++) {
                        imu[i].device_id = 0;
                        b0[i] = imu[i].rx_bytes;
                        m0[i] = imu[i].msg_other + imu[i].pkt_ok;
                        imu[i].send_cmd(0x00);
                    }
                    ThisThread::sleep_for(300ms);
                    printf("  %7d:", b);
                    for (int i = 0; i < N_IMU; i++)
                        printf("  %c bytes %lu msgs %lu id %08lX", 'A' + i, (unsigned long)(imu[i].rx_bytes - b0[i]),
                               (unsigned long)(imu[i].msg_other + imu[i].pkt_ok - m0[i]),
                               (unsigned long)imu[i].device_id);
                    printf("\r\n");
                }
                for (int i = 0; i < N_IMU; i++) imu[i].set_baud(IMU_BAUD[i]);
                next = Kernel::Clock::now();
            } else if (ch == 'c') {
                reset_all();
                for (int i = 0; i < N_IMU; i++) prev_ok[i] = prev_bytes[i] = 0;
                printf("-- 統計歸零 --\r\n");
            }
        }

        // 讀兩顆 IMU 最新資料
        ImuSample s[N_IMU];
        int nnew[N_IMU];
        for (int i = 0; i < N_IMU; i++) {
            nnew[i] = imu[i].fetch(s[i]);
            Stats& t = st[i];
            if (nnew[i] == 0) t.cycles_new0++;
            if (nnew[i] >= 2) t.cycles_new2++;
            if (nnew[i] > 0 && s[i].packet_counter >= 0) {
                if (t.last_pc >= 0) {
                    uint16_t gap = (uint16_t)(s[i].packet_counter - t.last_pc);
                    if (gap > (uint16_t)nnew[i]) t.lost += gap - nnew[i];
                }
                t.last_pc = s[i].packet_counter;
            }
            if (s[i].t_us != 0) {
                int32_t age_s = (int32_t)(now - s[i].t_us);  // 讀 now 之後才進來的封包會是負的
                uint32_t age = age_s > 0 ? age_s : 0;
                if (age > t.max_age_us) t.max_age_us = age;
            }
        }

        if (stream) {
            printf("D,%lu,%lu", (unsigned long)cycle, (unsigned long)((now - t0) / 1000));
            for (int i = 0; i < N_IMU; i++) {
                const ImuSample& x = s[i];
                printf(",%d,%ld,%.1f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
                       nnew[i], (long)x.packet_counter, x.t_us ? fmaxf((int32_t)(now - x.t_us), 0) / 1000.0f : -1.0f,
                       x.euler[0], x.euler[1], x.euler[2], x.accel[0], x.accel[1], x.accel[2],
                       x.gyro[0] * RAD2DEG, x.gyro[1] * RAD2DEG, x.gyro[2] * RAD2DEG);
            }
            printf("\r\n");
        }

        // 每秒統計
        if (cycle % LOOP_HZ == 0) {
            led = !led;
            if (!stream) {
                printf("[%lus]\r\n", (unsigned long)((now - t0) / 1000000));
                for (int i = 0; i < N_IMU; i++) {
                    uint32_t ok = imu[i].pkt_ok, bytes = imu[i].rx_bytes;
                    Stats& t = st[i];
                    printf("  %s %3lu pkt/s %5lu B/s | cks_err %lu len_err %lu lost %lu | "
                           "new0 %lu new2 %lu max_age %.1fms\r\n",
                           IMU_NAME[i], (unsigned long)(ok - prev_ok[i]), (unsigned long)(bytes - prev_bytes[i]),
                           (unsigned long)imu[i].pkt_bad_cksum, (unsigned long)imu[i].pkt_bad_len,
                           (unsigned long)t.lost, (unsigned long)t.cycles_new0, (unsigned long)t.cycles_new2,
                           t.max_age_us / 1000.0f);
                    if (imu[i].msg_other)
                        printf("    其他訊息 %lu 筆 (最後 MID 0x%02X)  DeviceID %08lX\r\n",
                               (unsigned long)imu[i].msg_other, imu[i].last_other_mid,
                               (unsigned long)imu[i].device_id);
                    if (bytes == prev_bytes[i]) {
                        printf("    -> 沒收到任何 byte：檢查 IMU TX 接到 RX 腳、GND、IMU 供電\r\n");
                    } else if (ok == prev_ok[i]) {
                        printf("    -> 有收到 byte 但沒有正確封包：檢查鮑率 (%d) 是否跟 IMU 一致\r\n", IMU_BAUD[i]);
                    } else {
                        const ImuSample& x = s[i];
                        printf("    pc %ld  R %7.2f P %7.2f Y %7.2f | a %6.2f %6.2f %6.2f | g %7.2f %7.2f %7.2f\r\n",
                               (long)x.packet_counter, x.euler[0], x.euler[1], x.euler[2], x.accel[0], x.accel[1],
                               x.accel[2], x.gyro[0] * RAD2DEG, x.gyro[1] * RAD2DEG, x.gyro[2] * RAD2DEG);
                    }
                    prev_ok[i] = ok;
                    prev_bytes[i] = bytes;
                    t.max_age_us = 0;
                }
            }
        }
    }
}
