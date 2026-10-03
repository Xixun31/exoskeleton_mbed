// =====================================================================
// MTi (Xsens MTi-1 series) SPI 單顆測試韌體
//
// 一次接一顆 IMU，開機後自動測試一次；之後從序列埠送 't' 可以再測一次。
// 搭配 python/imu/imu_spi_check.py 逐顆記錄六顆 IMU 的結果。
//
// 測試項目：
//   1. ProtInfo     : SPI 通訊是否正常 (fill word 應為 FA FF FF FF)
//   2. DeviceID     : 讀取 IMU 序號 (用來分辨是哪一顆)
//   3. 封包         : TEST_SECONDS 秒內的封包數、速率、checksum 錯誤
//   4. 加速度計     : 靜止時 |a| 應接近 9.81 m/s^2
//   5. 陀螺儀       : 靜止時平均值接近 0、雜訊小
//   6. 姿態角       : roll/pitch 應與加速度計算出的傾角一致
//
// 測試時請讓 IMU 靜止不動。
// =====================================================================
#include "mbed.h"
#include <cmath>

// --- SPI 接線 (與 mbed/imu_spi 相同，SPI2 在 Morpho 排針) ---
// MOSI: PB_15, MISO: PB_14, SCLK: PB_13, CS: PB_1
SPI spi(PB_15, PB_14, PB_13);
DigitalOut cs(PB_1, 1);
int spi_freq = 200000;  // 送 '1'~'4' 可切換：50k / 100k / 200k / 500k

// --- 測試參數 ---
const int TEST_SECONDS = 5;
const float GRAVITY = 9.81f;
const float ACC_TOL = 0.5f;        // |a| 允許誤差 (m/s^2)
const float GYRO_MEAN_TOL = 2.0f;  // 靜止時陀螺儀平均值上限 (deg/s)
const float GYRO_STD_TOL = 1.0f;   // 靜止時陀螺儀雜訊上限 (deg/s)
const float TILT_TOL = 3.0f;       // 姿態角與加速度計傾角的允許差 (deg)
const float MIN_RATE = 10.0f;      // 封包速率下限 (Hz)
const float RAD2DEG = 57.2957795f;

// --- MTi SPI opcode ---
const uint8_t OP_PROTINFO = 0x01;
const uint8_t OP_CONFIGPROT = 0x02;
const uint8_t OP_CONTROL = 0x03;
const uint8_t OP_PIPESTATUS = 0x04;
const uint8_t OP_NOTIFICATION = 0x05;
const uint8_t OP_MEASUREMENT = 0x06;

// --- Xbus message ID ---
const uint8_t MID_REQDID = 0x00;
const uint8_t MID_DEVICEID = 0x01;
const uint8_t MID_GOTOMEAS = 0x10;
const uint8_t MID_GOTOMEAS_ACK = 0x11;
const uint8_t MID_GOTOCONFIG = 0x30;
const uint8_t MID_GOTOCONFIG_ACK = 0x31;
const uint8_t MID_MTDATA2 = 0x36;
const uint8_t MID_ERROR = 0x42;
const uint8_t MID_REQFWREV = 0x12;
const uint8_t MID_FWREV = 0x13;
const uint8_t MID_REQOUTPUTCONFIG = 0xC0;
const uint8_t MID_OUTPUTCONFIG = 0xC1;

// 常見的資料種類 (XDI，低 4 bits 是格式旗標，比對時忽略)
const char* xdi_name(uint16_t xdi) {
    switch (xdi & 0xFFF0) {
        case 0x0810: return "Temperature";
        case 0x1020: return "PacketCounter";
        case 0x1060: return "SampleTimeFine";
        case 0x2010: return "Quaternion";
        case 0x2020: return "RotationMatrix";
        case 0x2030: return "EulerAngles";
        case 0x4010: return "DeltaV";
        case 0x4020: return "Acceleration";
        case 0x4030: return "FreeAcceleration";
        case 0x8020: return "RateOfTurn";
        case 0x8030: return "DeltaQ";
        case 0xC020: return "MagneticField";
        case 0xE020: return "StatusWord";
        default: return "?";
    }
}

uint8_t rx[256];
uint8_t fillword[4];

// ---------------- 低階 SPI ----------------
void send_opcode(uint8_t op) {
    fillword[0] = spi.write(op);
    for (int i = 0; i < 3; i++) {
        fillword[i + 1] = spi.write(i);
    }
}

void transfer(uint8_t op, const uint8_t* tx, uint8_t* out, int len) {
    cs = 0;
    send_opcode(op);
    for (int i = 0; i < len; i++) {
        uint8_t b = spi.write(tx ? tx[i] : 0x00);
        if (out) {
            out[i] = b;
        }
    }
    cs = 1;
    wait_us(20);
}

void pipe_status(uint16_t* notif, uint16_t* meas) {
    uint8_t b[4];
    transfer(OP_PIPESTATUS, nullptr, b, 4);
    *notif = b[0] | (b[1] << 8);
    *meas = b[2] | (b[3] << 8);
}

// Xbus checksum：0xFF (省略的 BID) + MID + LEN + DATA + CHK 加總為 0
bool checksum_ok(const uint8_t* msg, int len) {
    uint8_t sum = 0xFF;
    for (int i = 0; i < len; i++) {
        sum += msg[i];
    }
    return sum == 0;
}

void send_xbus(uint8_t mid, const uint8_t* data, uint8_t len) {
    uint8_t msg[32];
    msg[0] = mid;
    msg[1] = len;
    uint8_t sum = 0xFF + mid + len;
    for (int i = 0; i < len; i++) {
        msg[2 + i] = data[i];
        sum += data[i];
    }
    msg[2 + len] = (uint8_t)(-sum);
    transfer(OP_CONTROL, msg, nullptr, len + 3);
}

// 等待 notification pipe 出現指定 MID 的訊息，回傳 payload 長度 (-1 = 逾時)
int wait_notification(uint8_t want_mid, uint8_t* payload, int timeout_ms) {
    Timer t;
    t.start();
    while (t.elapsed_time() < std::chrono::milliseconds(timeout_ms)) {
        uint16_t n, m;
        pipe_status(&n, &m);
        if (n > 0 && n <= sizeof(rx)) {
            transfer(OP_NOTIFICATION, nullptr, rx, n);
            if (rx[0] == want_mid && checksum_ok(rx, n)) {
                int len = rx[1];
                if (payload) {
                    memcpy(payload, &rx[2], len);
                }
                return len;
            }
            if (rx[0] == MID_ERROR) {
                printf("    (IMU 回報錯誤碼 0x%02X)\r\n", rx[2]);
            } else {
                // 診斷用：印出不是在等的訊息
                printf("    (收到其他訊息 MID=0x%02X len=%d:", rx[0], rx[1]);
                for (int i = 2; i < n && i < 14; i++) {
                    printf(" %02X", rx[i]);
                }
                printf("%s)\r\n", checksum_ok(rx, n) ? "" : " checksum錯");
            }
        } else if (n > sizeof(rx) || m > sizeof(rx)) {
            printf("    (pipe status 異常 notif=%u meas=%u)\r\n", n, m);
        } else if (m > 0) {
            transfer(OP_MEASUREMENT, nullptr, rx, m);  // 丟掉量測資料
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

// ---------------- 統計 ----------------
struct Stat {
    double sum = 0, sum2 = 0;
    int n = 0;
    void add(double x) { sum += x; sum2 += x * x; n++; }
    double mean() const { return n ? sum / n : NAN; }
    double std() const { return n > 1 ? sqrt(fmax(0.0, sum2 / n - mean() * mean())) : NAN; }
};

const char* pf(bool ok) { return ok ? "PASS" : "FAIL"; }

// ---------------- 測試 ----------------
void run_test() {
    printf("\r\n=== IMU SPI TEST START ===\r\n");
    printf("    SPI clock %d Hz\r\n", spi_freq);

    // 1. ProtInfo
    uint8_t info[2];
    transfer(OP_PROTINFO, nullptr, info, 2);
    bool prot_ok = fillword[0] == 0xFA && fillword[1] == 0xFF && fillword[2] == 0xFF && fillword[3] == 0xFF;
    printf("[1] ProtInfo   fill word %02X %02X %02X %02X, version %02X %02X -> %s\r\n",
           fillword[0], fillword[1], fillword[2], fillword[3], info[0], info[1], pf(prot_ok));
    if (!prot_ok) {
        printf("    SPI 沒有回應：請檢查 IMU 電源、GND、MOSI/MISO/SCLK/CS 接線\r\n");
        printf("RESULT,pass=0,prot=0,fw=%02X%02X%02X%02X,did=00000000,rate=0,pkts=0,cksum_err=0,acc=0,"
               "acc_std=0,gx=0,gy=0,gz=0,gstd=0,roll=0,pitch=0,tilt_err=0\r\n",
               fillword[0], fillword[1], fillword[2], fillword[3]);
        printf("=== IMU SPI TEST END ===\r\n");
        return;
    }
    transfer(OP_CONFIGPROT, (const uint8_t[]){0x08}, nullptr, 1);  // DRDY 設定 (與原 MTi2 驅動相同)

    // 2. DeviceID (先進 Config 模式讀序號，再回到 Measurement 模式)
    send_xbus(MID_GOTOCONFIG, nullptr, 0);
    bool cfg_ok = wait_notification(MID_GOTOCONFIG_ACK, nullptr, 1000) >= 0;
    uint8_t did_buf[8] = {0};
    uint32_t did = 0;
    bool did_ok = false;
    if (cfg_ok) {
        send_xbus(MID_REQDID, nullptr, 0);
        did_ok = wait_notification(MID_DEVICEID, did_buf, 1000) == 4;
        if (did_ok) {
            did = ((uint32_t)did_buf[0] << 24) | (did_buf[1] << 16) | (did_buf[2] << 8) | did_buf[3];
        }

        // 診斷：韌體版本與輸出設定
        uint8_t buf[64];
        send_xbus(MID_REQFWREV, nullptr, 0);
        int fw_len = wait_notification(MID_FWREV, buf, 1000);
        if (fw_len >= 3) {
            printf("    韌體版本 %d.%d.%d\r\n", buf[0], buf[1], buf[2]);
        } else {
            printf("    韌體版本：無回應\r\n");
        }
        send_xbus(MID_REQOUTPUTCONFIG, nullptr, 0);
        int oc_len = wait_notification(MID_OUTPUTCONFIG, buf, 1000);
        if (oc_len >= 0) {
            printf("    輸出設定 (%d 項):", oc_len / 4);
            for (int i = 0; i + 3 < oc_len; i += 4) {
                uint16_t xdi = (buf[i] << 8) | buf[i + 1];
                uint16_t hz = (buf[i + 2] << 8) | buf[i + 3];
                printf(" %s(0x%04X)@%uHz", xdi_name(xdi), xdi, hz);
            }
            printf("\r\n");
        } else {
            printf("    輸出設定：無回應\r\n");
        }
    }
    send_xbus(MID_GOTOMEAS, nullptr, 0);
    bool meas_ok = wait_notification(MID_GOTOMEAS_ACK, nullptr, 1000) >= 0;
    printf("[2] DeviceID   %08lX (GoToConfig %s, ReqDID %s, GoToMeasurement %s) -> %s\r\n",
           (unsigned long)did, cfg_ok ? "ok" : "無回應", did_ok ? "ok" : "無回應",
           meas_ok ? "ok" : "無回應", pf(did_ok));

    // 3~6. 讀 TEST_SECONDS 秒量測資料
    Stat acc_mag, gx, gy, gz, roll, pitch, tilt_err;
    int pkts = 0, cksum_err = 0, bad = 0;
    bool has_euler = false, has_acc = false, has_gyro = false;
    uint16_t seen_xdi[16];
    int n_seen = 0;
    bool dumped = false;
    Timer t;
    t.start();
    while (t.elapsed_time() < std::chrono::seconds(TEST_SECONDS)) {
        uint16_t n, m;
        pipe_status(&n, &m);
        if (n > 0 && n <= sizeof(rx)) {
            transfer(OP_NOTIFICATION, nullptr, rx, n);
        }
        if (m == 0) {
            ThisThread::sleep_for(1ms);
            continue;
        }
        if (m > sizeof(rx)) {
            bad++;
            continue;
        }
        transfer(OP_MEASUREMENT, nullptr, rx, m);
        if (rx[0] != MID_MTDATA2 || rx[1] + 3 > m) {
            bad++;
            continue;
        }
        if (!checksum_ok(rx, rx[1] + 3)) {
            cksum_err++;
            continue;
        }
        pkts++;
        if (!dumped) {
            dumped = true;
            printf("    第一個封包 (%d bytes):", rx[1] + 3);
            for (int k = 0; k < rx[1] + 3 && k < 24; k++) {
                printf(" %02X", rx[k]);
            }
            printf("%s\r\n", rx[1] + 3 > 24 ? " ..." : "");
        }

        float e[3] = {0}, a[3] = {0}, g[3] = {0};
        bool fe = false, fa = false, fg = false;
        int i = 2, end = 2 + rx[1];
        while (i + 3 <= end) {
            uint16_t xdi = (rx[i] << 8) | rx[i + 1];
            int size = rx[i + 2];
            const uint8_t* c = &rx[i + 3];
            if (i + 3 + size > end) {
                break;
            }
            bool known = false;
            for (int k = 0; k < n_seen; k++) {
                known |= seen_xdi[k] == xdi;
            }
            if (!known && n_seen < 16) {
                seen_xdi[n_seen++] = xdi;
            }
            if ((xdi & 0xFFF0) == 0x2030 && size == 12) {
                for (int k = 0; k < 3; k++) e[k] = be_float(c + 4 * k);
                fe = true;
            } else if ((xdi & 0xFFF0) == 0x4020 && size == 12) {
                for (int k = 0; k < 3; k++) a[k] = be_float(c + 4 * k);
                fa = true;
            } else if ((xdi & 0xFFF0) == 0x8020 && size == 12) {
                for (int k = 0; k < 3; k++) g[k] = be_float(c + 4 * k) * RAD2DEG;
                fg = true;
            }
            i += 3 + size;
        }
        has_euler |= fe;
        has_acc |= fa;
        has_gyro |= fg;
        if (fa) {
            acc_mag.add(sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));
        }
        if (fg) {
            gx.add(g[0]);
            gy.add(g[1]);
            gz.add(g[2]);
        }
        if (fe) {
            roll.add(e[0]);
            pitch.add(e[1]);
        }
        if (fe && fa) {
            // 靜止時由重力算出的傾角
            float r_acc = atan2f(a[1], a[2]) * RAD2DEG;
            float p_acc = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2])) * RAD2DEG;
            float dr = fmodf(e[0] - r_acc + 540.0f, 360.0f) - 180.0f;
            tilt_err.add(fmaxf(fabsf(dr), fabsf(e[1] - p_acc)));
        }
    }
    float secs = t.elapsed_time().count() / 1e6f;
    float rate = pkts / secs;

    printf("    封包內容:");
    for (int k = 0; k < n_seen; k++) {
        printf(" %s(0x%04X)", xdi_name(seen_xdi[k]), seen_xdi[k]);
    }
    printf("%s\r\n", n_seen ? "" : " (沒有收到封包)");

    bool rate_ok = rate >= MIN_RATE && cksum_err == 0;
    printf("[3] 封包       %d 個 / %.1f s = %.1f Hz, checksum 錯誤 %d, 格式錯誤 %d -> %s\r\n",
           pkts, secs, rate, cksum_err, bad, pf(rate_ok));

    bool acc_ok = has_acc && fabs(acc_mag.mean() - GRAVITY) < ACC_TOL;
    printf("[4] 加速度計   |a| = %.3f m/s^2 (std %.3f)%s -> %s\r\n",
           acc_mag.mean(), acc_mag.std(), has_acc ? "" : " 封包中沒有加速度資料", pf(acc_ok));

    double gstd = fmax(gx.std(), fmax(gy.std(), gz.std()));
    bool gyro_ok = has_gyro && fabs(gx.mean()) < GYRO_MEAN_TOL && fabs(gy.mean()) < GYRO_MEAN_TOL &&
                   fabs(gz.mean()) < GYRO_MEAN_TOL && gstd < GYRO_STD_TOL;
    printf("[5] 陀螺儀     平均 %.2f %.2f %.2f deg/s, 最大 std %.3f deg/s%s -> %s\r\n",
           gx.mean(), gy.mean(), gz.mean(), gstd, has_gyro ? "" : " 封包中沒有陀螺儀資料", pf(gyro_ok));

    bool euler_ok = has_euler && has_acc && tilt_err.mean() < TILT_TOL;
    printf("[6] 姿態角     roll %.2f pitch %.2f deg, 與加速度計傾角差 %.2f deg%s -> %s\r\n",
           roll.mean(), pitch.mean(), tilt_err.mean(), has_euler ? "" : " 封包中沒有姿態角資料",
           pf(euler_ok));

    bool all_ok = prot_ok && did_ok && rate_ok && acc_ok && gyro_ok && euler_ok;
    printf("RESULT,pass=%d,prot=%d,fw=FAFFFFFF,did=%08lX,rate=%.1f,pkts=%d,cksum_err=%d,acc=%.3f,acc_std=%.3f,"
           "gx=%.2f,gy=%.2f,gz=%.2f,gstd=%.3f,roll=%.2f,pitch=%.2f,tilt_err=%.2f\r\n",
           all_ok, prot_ok, (unsigned long)did, rate, pkts, cksum_err, acc_mag.mean(), acc_mag.std(),
           gx.mean(), gy.mean(), gz.mean(), gstd, roll.mean(), pitch.mean(), tilt_err.mean());
    printf("總結：%s\r\n", all_ok ? "全部通過" : "有項目未通過");
    printf("=== IMU SPI TEST END ===\r\n");
}

int main() {
    spi.format(8, 3);
    spi.frequency(spi_freq);
    ThisThread::sleep_for(3000ms);  // 等 IMU 開機完成 (太早測試會讀到亂碼)
    printf("\r\nMTi SPI 單顆測試韌體 (送 't' 重新測試)\r\n");

    // 用 readable() 輪詢輸入，不要把 console 設成 non-blocking (會連輸出一起變成 non-blocking 而掉字)
    FileHandle* console = mbed_file_handle(STDIN_FILENO);

    run_test();
    while (true) {
        char c;
        if (console->readable() && console->read(&c, 1) == 1) {
            if (c == 't' || c == 'T') {
                run_test();
            } else if (c >= '1' && c <= '4') {
                const int freqs[] = {50000, 100000, 200000, 500000};
                spi_freq = freqs[c - '1'];
                spi.frequency(spi_freq);
                printf("SPI clock 改為 %d Hz\r\n", spi_freq);
            }
        }
        ThisThread::sleep_for(20ms);
    }
}
