// =====================================================================
// 2 顆 AS5145 (daisy chain 串接) + 1 顆 SPI IMU + 2 顆 UART IMU (Xsens MTi) 以 100Hz 同步讀取，
// 加上左右腳足底感測器 (BLE → ESP32 → UART)，全部用同一個 MCU µs 時鐘打時間戳，存到 microSD
//
// 每個裝置都有自己的 time code (MCU 時鐘，秒，µs 解析度)，可以檢查時間有沒有對齊：
//   enc_t_s  : encoder CS 拉 Low 的時間 (AS5145 在 CSn 下降緣鎖存角度 = encoder 取樣時間)
//   imu_t_s  : IMU 新封包出現的時間 (DRDY 模式 = DRDY 上升緣；輪詢模式 = 偵測到新封包的那次輪詢，最多晚 1ms)
//   imu_stf  : IMU 自己的時鐘 SampleTimeFine (0.1ms)，可跟 imu_t_s 比較 IMU / MCU 時鐘漂移
//   imuA_t_s / imuB_t_s : UART IMU 最新封包第一個 byte 進來的時間 (= IMU 開始送這包的時間)
//   imuA_stf / imuB_stf : UART IMU 的 SampleTimeFine (IMU 有輸出才有，否則 0)
//   foot t_rx_s : STM32 收到足底封包最後一個 byte 的時間 (另存 runXXX_foot.csv)
//   foot esp_us : ESP32 收到 BLE 封包的 ESP32 時鐘 (micros())，可跟 t_rx_s 比較 UART 延遲與 ESP32 時鐘漂移
//   tools/check_time_align.py 會讀兩個 CSV，算出各裝置的取樣間隔、漏包、時鐘漂移與延遲抖動
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
//  [足底感測器 ESP32, USART1 115200]
//   ESP32 GPIO17 (TX) → PA_10 (RX) CN10-33     ESP32 GND → GND (CN10-9 / CN7-19 等，一定要共地)
//   PA_9 (TX) CN10-21 保留不接
//  [UART IMU A, USART3 921600]  IMU TX → PC_5 (RX) CN10-6    IMU RX ← PB_10 (TX) CN10-25
//  [UART IMU B, UART4  921600]  IMU TX → PA_1 (RX) CN7-30    IMU RX ← PA_0  (TX) CN7-28
//   兩顆 IMU 的 GND 都要接板子 GND；鮑率要跟 IMU 設定一樣 (UIMU_BAUD)，不知道可用 imu_uart_dual_100hz 的 's' 掃描
//  電源：3V3 CN7-16、5V CN7-18、GND CN7-8 / CN7-19 / CN7-20 / CN10-9 / CN10-20
//
//  AS5145 daisy chain：
//   CLK、CSn 兩顆並接；encoder1 DO → MCU MISO；encoder2 DO → encoder1 PROG；encoder2 PROG → GND
//   資料順序：encoder1 在前。每次讀 40 bit = 起始 1 bit + [enc1 18 bit] + dummy 1 bit + [enc2 18 bit] + 2 bit 不用
//   每顆 18 bit = D11..D0, OCF, COF, LIN, MagINC, MagDEC, Parity(偶同位)
//
// ---------------- 操作 ----------------
//  1. 開機後持續檢查所有裝置 (每秒一次)：encoder 通訊正常、SPI IMU / UART IMU A / B 都約 100 包/秒、
//     左右腳足底都有收到、SD 卡可用。全部 OK = 準備好 → LD2 綠燈恆亮 (沒準備好 = 熄滅，序列埠會印缺哪一項)
//  2. 準備好之後按一下藍色 B1 (放開時開始)：開新檔 runXXX.csv / runXXX_foot.csv (不覆蓋舊檔)，
//     先記錄 3 秒原點 (phase = 1，請保持不動，LD2 快閃)，結束時把每個通道 3 秒的平均存成 runXXX_origin.csv
//  3. 原點記錄完自動進入測量 (phase = 2，LD2 慢閃)
//  4. 測量中再按一下 B1 = 停止並關檔 (可以拔卡)，回到步驟 1
//  原點期間 encoder 或 IMU 姿態變化超過 2 deg 時，序列埠會印 WARN (表示有動到，建議重來)
//  序列埠 115200：每秒印狀態 (enc_err = 通訊錯誤、mag_warn = 磁場警告、UIMU = UART IMU 每秒封包數)。
//  指令 's' / 'S' 切換即時資料輸出 10Hz / 50Hz (python/imu/live_plot_enc_imu.py 畫圖)、
//  'g' 開始 (沒準備好也強制開始)、'x' 停止、'i' 重新初始化 SPI IMU、'z' encoder 歸零 (存 flash，只能在沒紀錄時用)
//  REQ_* 常數可以設定哪些裝置是「準備好」的必要條件 (例如今天沒接足底就把 REQ_FOOT 改 false)
//
// ---------------- CSV 欄位 (runXXX.csv，100Hz) ----------------
//  cycle,phase,t_s,trig,enc_t_s,
//  enc1_raw,enc1_deg,enc1_stat,enc1_err,enc2_raw,enc2_deg,enc2_stat,enc2_err,
//  imu_t_s,imu_new,imu_bad,imu_pc,imu_stf,roll,pitch,yaw,gx,gy,gz,ax,ay,az,
//  imuA_t_s,imuA_new,imuA_bad,imuA_pc,imuA_stf,imuA_roll,...,imuA_az,   (UART IMU A，欄位同 SPI IMU)
//  imuB_t_s,imuB_new,imuB_bad,imuB_pc,imuB_stf,imuB_roll,...,imuB_az,   (UART IMU B)
//  footL_cnt,footR_cnt,busy_us,dropped
//   phase    : 1 = 原點 (前 3 秒，保持不動)、2 = 測量
//   t_s      : 週期開始的 MCU 時間 (秒，µs 解析度)
//   imuA_new / imuB_new : 這個週期收到幾個新封包。UART IMU 自己以 100Hz 送，跟 SPI IMU 不同步，
//              偶爾 0 (沿用上一筆) 或 2 (只留最新那筆)，用 imuX_pc 可以看出來
//   imuA_bad / imuB_bad : 這個週期 checksum / 長度錯誤的封包數
//   enc_t_s / imu_t_s : 見上面 time code 說明 (IMU 這週期沒有新封包時沿用上一筆)
//   footL_cnt / footR_cnt : 到這個週期為止收到的左 / 右腳封包數 (= runXXX_foot.csv 的 seq，用來對照)
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
//
// ---------------- CSV 欄位 (runXXX_foot.csv，每收到一包足底資料一行) ----------------
//  seq,foot,phase,t_rx_s,cycle,esp_us,esp_seq,P1..P18,total_g,cop_valid,cop_x_mm,cop_y_mm
//   seq      : 該腳第幾包 (左右各自從 1 開始，對應主檔 footL_cnt / footR_cnt)
//   phase    : 1 = 原點、2 = 測量 (依 t_rx_s 判斷)
//   foot     : L / R
//   t_rx_s   : STM32 收到最後一個 byte 的 MCU 時間 (與主檔 t_s 同一個時鐘)
//   cycle    : 收到時主檔最新的週期編號 (主檔該 cycle 那一行之後收到)
//   esp_us   : ESP32 收到 BLE 封包時的 micros() (32 bit，約 71 分鐘繞回)；舊版 ESP32 韌體沒有時間戳 → 空白
//   esp_seq  : ESP32 對該腳的封包編號 (16 bit)，跳號 = UART 掉包；舊版韌體空白
//
// ---------------- runXXX_origin.csv (原點 3 秒的統計，每個通道一行) ----------------
//  channel,mean,min,max,n,t_first_s,t_last_s
//   channel  : enc1_deg、imu_roll…imu_az、imuA_*、imuB_*、footL_P1…footL_total_g、footR_*
//   mean     : 3 秒平均 (角度有處理 ±180 繞回)；min / max = 3 秒內的範圍 (看有沒有動到)
//   n        : 用了幾筆 (IMU 只算新封包、encoder 只算通訊正確的)
//   t_first_s / t_last_s : 第一筆 / 最後一筆的 time code (該裝置自己的時間戳，跟主檔同一個時鐘)
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

// SPI3 在 APB1 (45MHz) 只能除 2 的次方：12MHz → 實際 11.25MHz (原本 4MHz → 實際只有 2.8MHz)。
// 2026-11-06 實測：2.8MHz 寫檔佔主執行緒 ~470ms/s (SD 卡一慢就塞滿佇列 dropped)，11.25MHz ~300ms/s。
// 杜邦線太長、mount 失敗或 CSV 出現亂碼時改 8000000 (5.6MHz) 或 1000000
const int SD_SPI_FREQ = 12000000;
SDBlockDevice sd(PC_12, PC_11, PC_10, PD_2, SD_SPI_FREQ);  // mosi, miso, sclk, cs (SPI3)
FATFileSystem fs("fs");

DigitalIn user_button(BUTTON1);  // PC13，按下 = 0
DigitalOut led(LED1);            // PA5 (LD2)

// 足底感測器：ESP32 GPIO17 TX → PA_10。鮑率要跟 ESP32 程式的 UART_BAUD 一樣
const int FOOT_BAUD = 115200;
UnbufferedSerial foot_uart(PA_9, PA_10, FOOT_BAUD);  // tx (不接), rx

// UART IMU：A = USART3 (PB_10 TX, PC_5 RX)、B = UART4 (PA_0 TX, PA_1 RX)。鮑率要跟各 IMU 的設定一樣
const int N_UIMU = 2;
const int UIMU_BAUD[N_UIMU] = {115200, 115200};

// 「準備好」(LD2 恆亮) 需要哪些裝置正常；今天沒接的裝置改 false
const bool REQ_ENC = true, REQ_SPI_IMU = true, REQ_UART_IMU = true, REQ_FOOT = true;
const int READY_MIN_RATE = 90;  // IMU / 週期每秒至少幾筆才算正常

const auto PERIOD = 10ms;  // 100Hz
const uint64_t ORIGIN_US = 3000000;  // 原點記錄 3 秒
const float ORIGIN_MOVE_DEG = 2.0f;  // 原點期間角度變化超過這個值就警告
const float RAD2DEG = 57.2957795f;
const char* LOG_DIR = "/fs/exoskeleton_project_record/enc_imu_foot";

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
    uint64_t enc_t_us;  // encoder CS 下降緣 (AS5145 鎖存角度的時間)
    uint64_t imu_t_us;  // 最新 IMU 封包出現的時間
    uint8_t trig;
    uint8_t phase;      // 0 = 沒在紀錄、1 = 原點、2 = 測量
    EncData enc[N_ENC];
    ImuData imu;
    ImuData uimu[2];         // UART IMU A / B
    uint64_t uimu_t_us[2];   // UART IMU 最新封包第一個 byte 的時間
    uint32_t foot_cnt[2];  // 到目前為止左 / 右腳收到幾包
    uint16_t busy;
    uint32_t dropped;
};
Mail<Sample, 200> mail;  // 2 秒緩衝 (SD 卡偶爾卡 100ms 以上)
volatile uint32_t dropped = 0;
volatile uint32_t q_depth = 0, q_max = 0;  // 佇列目前 / 這一秒最多積了幾筆 (到 200 就會 dropped)
// 紀錄狀態：main 執行緒設定 (先設 origin_end_us 再設 rec_on)，取樣執行緒依週期開始時間決定 phase
volatile bool rec_on = false;
volatile uint64_t origin_end_us = 0;
Timer clock_us;

uint64_t now_us() { return clock_us.elapsed_time().count(); }

// ---------------- 足底感測器 (UART 中斷收資料) ----------------
// ESP32 送來的封包有兩種，都接受：
//  舊版 (39 bytes)：0xAA, foot(0x01 L / 0x02 R), 18 點壓力 (High, Low)×18, checksum (前 38 bytes 加總低 8 位)
//  新版 (47 bytes)：0xAB, [上面整個 39 bytes], esp_us (uint32 little-endian), esp_seq (uint16 LE),
//                   checksum (前 46 bytes 加總低 8 位)
const int FOOT_N = 18;
const int FOOT_FRAME = 39, FOOT_FRAME_TS = 47;
struct FootFrame {
    uint32_t seq;      // 該腳第幾包 (從 1 開始)
    uint32_t cycle;    // 收到時最新的主週期編號
    uint64_t t_rx_us;  // 收到最後一個 byte 的 MCU 時間
    uint32_t esp_us;
    uint16_t esp_seq;
    uint8_t foot;    // 0 = L, 1 = R
    bool has_esp;    // 新版封包才有 ESP32 時間戳
    uint16_t p[FOOT_N];
};
Mail<FootFrame, 64> foot_mail;
volatile uint32_t foot_cnt[2] = {};
volatile uint32_t foot_bad = 0, foot_drop = 0;
volatile uint32_t cur_cycle = 0;

uint8_t foot_buf[FOOT_FRAME_TS];
int foot_idx = 0, foot_len = 0;

bool sum_ok(const uint8_t* b, int n) {  // b[n] = b[0..n-1] 加總低 8 位
    uint8_t s = 0;
    for (int k = 0; k < n; k++) s += b[k];
    return s == b[n];
}

void foot_frame_done(uint64_t t) {
    const uint8_t* f = foot_len == FOOT_FRAME ? foot_buf : foot_buf + 1;  // 內層 39 bytes
    bool ok = sum_ok(f, 38) && (foot_len == FOOT_FRAME || sum_ok(foot_buf, 46));
    if (!ok) {
        foot_bad++;
        return;
    }
    int side = f[1] == 0x01 ? 0 : 1;
    uint32_t seq = ++foot_cnt[side];
    FootFrame* p = foot_mail.try_alloc();
    if (!p) {
        foot_drop++;
        return;
    }
    p->seq = seq;
    p->cycle = cur_cycle;
    p->t_rx_us = t;
    p->foot = side;
    p->has_esp = foot_len == FOOT_FRAME_TS;
    p->esp_us = p->has_esp ? (foot_buf[40] | (foot_buf[41] << 8) | (foot_buf[42] << 16) | ((uint32_t)foot_buf[43] << 24)) : 0;
    p->esp_seq = p->has_esp ? (foot_buf[44] | (foot_buf[45] << 8)) : 0;
    for (int i = 0; i < FOOT_N; i++) p->p[i] = (f[2 + 2 * i] << 8) | f[3 + 2 * i];
    foot_mail.put(p);
}

// 一次處理一個 byte (在 UART 中斷裡跑，很短)
void foot_parse(uint8_t b, uint64_t t) {
    if (foot_idx == 0) {
        if (b == 0xAA || b == 0xAB) {
            foot_buf[0] = b;
            foot_len = b == 0xAA ? FOOT_FRAME : FOOT_FRAME_TS;
            foot_idx = 1;
        }
        return;
    }
    // 檢查表頭：舊版 [AA, 01/02]；新版 [AB, AA, 01/02]
    int hdr = foot_len == FOOT_FRAME ? foot_idx : foot_idx - 1;  // 在內層 39 bytes 的位置
    bool bad_hdr = (hdr == 0 && b != 0xAA) || (hdr == 1 && b != 0x01 && b != 0x02);
    if (bad_hdr) {
        foot_idx = 0;
        foot_parse(b, t);  // 這個 byte 可能是下一包的開頭
        return;
    }
    foot_buf[foot_idx++] = b;
    if (foot_idx == foot_len) {
        foot_frame_done(t);
        foot_idx = 0;
    }
}

void on_foot_rx() {
    uint64_t t = now_us();
    uint8_t c;
    while (foot_uart.readable()) {
        if (foot_uart.read(&c, 1) == 1) foot_parse(c, t);
    }
}

// 18 個感測區域幾何中心 (AutoCAD 世界座標，mm)，與 Foot_sensor/main.cpp 相同
const float FOOT_X[FOOT_N] = {1943.9325f, 1940.0133f, 1973.6715f, 1972.9752f, 1965.7372f, 1957.7049f,
                              1963.0637f, 1955.6792f, 1995.8721f, 1996.5690f, 1983.1920f, 1980.9491f,
                              1982.3130f, 1970.9490f, 2000.9966f, 2003.7015f, 2001.4963f, 1988.7695f};
const float FOOT_Y[FOOT_N] = {2600.1604f, 2637.9524f, 2444.2083f, 2482.2769f, 2521.7471f, 2561.9473f,
                              2599.0265f, 2642.2744f, 2444.2212f, 2482.2769f, 2520.6611f, 2559.8462f,
                              2599.0265f, 2641.6030f, 2521.0309f, 2560.0963f, 2597.5737f, 2633.6218f};
const float FOOT_ORIGIN_X = 1984.7721f, FOOT_ORIGIN_Y = 2415.3761f;  // 之後決定腳跟原點後改這裡
const float FOOT_FORCE_THRESHOLD = 50.0f;                            // 空載雜訊門檻 (g)

// 解析一個 MTData2 訊息的資料區 (一串 XDI + size + content)，更新最新值。SPI / UART IMU 共用
void parse_mtdata2(const uint8_t* d, int n, ImuData& s) {
    int i = 0;
    while (i + 3 <= n) {
        uint16_t xdi = (d[i] << 8) | d[i + 1];
        int size = d[i + 2];
        const uint8_t* c = &d[i + 3];
        if (i + 3 + size > n) break;
        switch (xdi & 0xFFF0) {
            case 0x1020: if (size == 2) s.pc = (c[0] << 8) | c[1]; break;
            case 0x1060: if (size == 4) s.stf = ((uint32_t)c[0] << 24) | (c[1] << 16) | (c[2] << 8) | c[3]; break;
            case 0x2030: if (size == 12) for (int k = 0; k < 3; k++) s.euler[k] = be_float(c + 4 * k); break;
            case 0x4020: if (size == 12) for (int k = 0; k < 3; k++) s.acc[k] = be_float(c + 4 * k); break;
            case 0x8020: if (size == 12) for (int k = 0; k < 3; k++) s.gyr[k] = be_float(c + 4 * k) * RAD2DEG; break;
        }
        i += 3 + size;
    }
}

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
        parse_mtdata2(&buf[off + 2], buf[off + 1], s);
        off += mlen;
    }
}

// ---------------- UART IMU ----------------
// 921600 baud 每 10.8us 一個 byte，STM32 USART 沒有 FIFO：用 RX 中斷逐 byte 收的話，只要其他中斷 / critical section
// 擋住 >10us 就會 overrun 掉 byte (實測跟 encoder / SPI IMU / SD 一起跑時掉 ~3% byte，大部分封包 checksum 錯)。
// 所以改成 DMA 循環接收 (硬體直接搬進 RAM，不怕中斷延遲)，再用 IDLE-line 中斷 (每包收完、線路空閒 1 byte 時間)
// 把 DMA buffer 裡的新 byte 拿出來解析 Xbus 封包：FA FF MID LEN payload CHECKSUM (FF+MID+LEN+payload+CHECKSUM 低 8 bit = 0)。
// 每個 byte 的時間 = 處理當下時間 − 之後還有幾個 byte × 每 byte 傳輸時間，所以中斷晚到不影響時間戳。
// 取樣執行緒每個週期用 fetch() 拿最新一筆 (fetch 也會先把 buffer 處理完)
//   USART3_RX = DMA1 Stream1 Channel4、UART4_RX = DMA1 Stream2 Channel4 (RM0390 Table 28；mbed 沒用到這兩個 stream)
class MtiUart {
public:
    volatile uint32_t rx_bytes = 0, pkt_ok = 0, pkt_bad = 0;

    MtiUart(PinName tx, PinName rx, int baud, USART_TypeDef* usart, DMA_Stream_TypeDef* dma, uint32_t dma_ch,
            IRQn_Type irqn, int slot)
        : serial(tx, rx, baud), usart(usart), dma(dma), dma_ch(dma_ch), irqn(irqn), slot(slot),
          byte_ns(10000000000ULL / baud) {}

    void start() {
        if (!dma_on) init_dma();
        core_util_critical_section_enter();
        rd = (DMA_BUF - dma->NDTR) & (DMA_BUF - 1);  // 丟掉停止期間收到的 byte，重新同步
        state = S_PRE;
        core_util_critical_section_exit();
        usart->CR1 |= USART_CR1_IDLEIE;
        running = true;
    }
    void stop() {
        running = false;
        usart->CR1 &= ~USART_CR1_IDLEIE;
    }

    // 送無資料的指令 (例如 0x10 GoToMeasurement)
    void send_cmd(uint8_t m) {
        uint8_t msg[5] = {0xFA, 0xFF, m, 0x00, (uint8_t)(0x100 - ((0xFF + m) & 0xFF))};
        serial.write(msg, 5);
    }

    // 複製最新一筆；d.n_new / d.bad = 上次呼叫到現在的新封包數 / 錯誤封包數，t = 最新封包第一個 byte 的時間
    void fetch(ImuData& d, uint64_t& t) {
        core_util_critical_section_enter();
        if (running) drain(now_us());
        d = latest;
        t = latest_t;
        uint32_t n = pkt_ok - fetched_ok, b = pkt_bad - fetched_bad;
        fetched_ok = pkt_ok;
        fetched_bad = pkt_bad;
        core_util_critical_section_exit();
        d.n_new = n > 255 ? 255 : n;
        d.bad = b > 255 ? 255 : b;
    }

    static void isr_slot0() { inst[0]->on_irq(); }
    static void isr_slot1() { inst[1]->on_irq(); }

private:
    static const uint32_t DMA_BUF = 512;  // 2 的次方；約 7 包，IDLE 中斷每包都會清
    static MtiUart* inst[2];
    UnbufferedSerial serial;  // 只用來設定腳位 / 鮑率和送指令 (TX 是 polling)，不掛 RX callback
    USART_TypeDef* usart;
    DMA_Stream_TypeDef* dma;
    uint32_t dma_ch;
    IRQn_Type irqn;
    int slot;
    uint32_t byte_ns;  // 一個 byte (10 bit) 的傳輸時間
    volatile uint8_t dma_buf[DMA_BUF];
    uint32_t rd = 0;
    bool dma_on = false;
    volatile bool running = false;

    void init_dma() {
        // 沒掛 RX callback 時 mbed 不會鎖 deep sleep；deep sleep 會停掉 USART / DMA 時脈
        sleep_manager_lock_deep_sleep();
        __HAL_RCC_DMA1_CLK_ENABLE();
        dma->CR &= ~DMA_SxCR_EN;
        while (dma->CR & DMA_SxCR_EN) {}
        // 清掉這個 stream 的旗標 (LIFCR: stream0~3、HIFCR: stream4~7)
        uint32_t idx = ((uint32_t)dma - (uint32_t)DMA1_Stream0) / 0x18;
        static const uint8_t shift[4] = {0, 6, 16, 22};
        if (idx < 4) DMA1->LIFCR = 0x3Du << shift[idx];
        else DMA1->HIFCR = 0x3Du << shift[idx - 4];
        dma->PAR = (uint32_t)&usart->DR;
        dma->M0AR = (uint32_t)dma_buf;
        dma->NDTR = DMA_BUF;
        dma->FCR = 0;  // direct mode
        dma->CR = (dma_ch << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 | DMA_SxCR_MINC | DMA_SxCR_CIRC;  // 週邊 -> 記憶體, 8 bit
        dma->CR |= DMA_SxCR_EN;
        usart->CR3 |= USART_CR3_DMAR;
        inst[slot] = this;
        NVIC_SetVector(irqn, slot == 0 ? (uint32_t)&isr_slot0 : (uint32_t)&isr_slot1);
        NVIC_ClearPendingIRQ(irqn);
        // 優先權比足底 UART (USART1，mbed 預設 0) 低，足底的 RX 中斷可以插隊
        NVIC_SetPriority(irqn, 2);
        NVIC_EnableIRQ(irqn);
        dma_on = true;
    }

    void on_irq() {
        uint32_t sr = usart->SR;
        if (sr & USART_SR_IDLE) {
            (void)usart->DR;  // SR 再讀 DR 才會清 IDLE (此時 RX 是空的，資料已被 DMA 拿走)
            if (running) drain(now_us());
        }
    }

    // 處理 DMA buffer 裡還沒解析的 byte；t = 現在時間。呼叫端要確保不會跟 ISR 同時跑。
    // 這段在中斷 / critical section 裡跑，要很短 (會擋住足底 UART 中斷，115200 只能擋 ~170us)：
    // 只有封包開頭 0xFA 才算時間戳 (32 bit 整數)，其他 byte 只跑狀態機
    void drain(uint64_t t) {
        uint32_t wr = (DMA_BUF - dma->NDTR) & (DMA_BUF - 1);
        uint32_t n = (wr - rd) & (DMA_BUF - 1);
        uint32_t r = rd;
        for (uint32_t rem = n; rem > 0; rem--) {
            uint8_t b = dma_buf[r];
            r = (r + 1) & (DMA_BUF - 1);
            if (b == 0xFA && (state == S_PRE || state == S_BID))
                t_start = t - (rem * byte_ns) / 1000;  // 這個 byte 收完的大約時間
            feed(b);
        }
        rd = r;
        rx_bytes += n;
    }

    ImuData latest = {};
    uint64_t latest_t = 0, t_start = 0;
    uint32_t fetched_ok = 0, fetched_bad = 0;
    enum { S_PRE, S_BID, S_MID, S_LEN, S_DATA, S_CKS } state = S_PRE;
    uint8_t mid = 0, len = 0, idx = 0, sum = 0;
    uint8_t buf[256];

    void feed(uint8_t b) {  // t_start 由 drain() 在 0xFA 時設定
        switch (state) {
            case S_PRE:
                if (b == 0xFA) state = S_BID;
                break;
            case S_BID:
                if (b == 0xFF) { sum = b; state = S_MID; }
                else if (b != 0xFA) state = S_PRE;
                break;
            case S_MID: mid = b; sum += b; state = S_LEN; break;
            case S_LEN:
                len = b; sum += b; idx = 0;
                if (len == 0xFF) { pkt_bad++; state = S_PRE; }  // extended length 不支援
                else state = len == 0 ? S_CKS : S_DATA;
                break;
            case S_DATA:
                buf[idx++] = b; sum += b;
                if (idx >= len) state = S_CKS;
                break;
            case S_CKS:
                sum += b;
                if (sum != 0) pkt_bad++;
                else if (mid == MID_MTDATA2) {
                    parse_mtdata2(buf, len, latest);
                    latest_t = t_start;
                    pkt_ok++;
                }
                state = S_PRE;
                break;
        }
    }

};
MtiUart* MtiUart::inst[2] = {nullptr, nullptr};

MtiUart uimu[N_UIMU] = {{PB_10, PC_5, UIMU_BAUD[0], USART3, DMA1_Stream1, 4, USART3_IRQn, 0},
                        {PA_0, PA_1, UIMU_BAUD[1], UART4, DMA1_Stream2, 4, UART4_IRQn, 1}};
const char UIMU_NAME[N_UIMU] = {'A', 'B'};

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
uint64_t read_encoder_bits(uint64_t* t_latch = nullptr) {
    char tx[5] = {}, b[5];
    if (t_latch) *t_latch = now_us();
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

volatile uint64_t drdy_t_us = 0;
void on_drdy() {
    drdy_t_us = now_us();
    sample_flag.set(FLAG_DRDY);
}
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
        uint64_t imu_seen = 0;  // IMU 新封包出現的時間 (0 = 不知道，用讀取時間)
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
                imu_seen = drdy_t_us;
            }
        } else if (use_poll) {
            sample_flag.wait_any(FLAG_TICK);
            uint16_t n, m;
            imu_seen = now_us();
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
        s.cycle = cycle;
        cur_cycle = cycle++;
        s.t_us = t0;
        s.trig = trig;
        uint64_t v = read_encoder_bits(&s.enc_t_us);
        for (int i = 0; i < N_ENC; i++) parse_encoder(v, i, s.enc[i]);
        s.imu.n_new = 0;
        s.imu.bad = 0;
        uint64_t t_imu_read = now_us();
        read_imu(s.imu);
        if (s.imu.n_new > 0) s.imu_t_us = (trig != 0 && imu_seen) ? imu_seen : t_imu_read;  // 沒新封包就沿用上一筆
        for (int k = 0; k < N_UIMU; k++) uimu[k].fetch(s.uimu[k], s.uimu_t_us[k]);
        s.phase = !rec_on ? 0 : (t0 < origin_end_us ? 1 : 2);
        for (int i = 0; i < 2; i++) s.foot_cnt[i] = foot_cnt[i];
        s.busy = (uint16_t)(now_us() - t0);
        s.dropped = dropped;
        last = s;

        Sample* p = mail.try_alloc();
        if (p) {
            *p = s;
            mail.put(p);
            uint32_t d = core_util_atomic_incr_u32(&q_depth, 1);
            if (d > q_max) q_max = d;
        } else {
            dropped++;
        }
    }
}

// ---------------- SD 紀錄 ----------------
FILE* log_file = nullptr;   // runXXX.csv      (encoder + IMU，100Hz)
FILE* foot_file = nullptr;  // runXXX_foot.csv (足底，每包一行)
bool sd_mounted = false;
bool sd_error = false;
char log_path[80] = {};
char foot_path[80] = {};
static char sd_stdio_buffer[8192];
static char foot_stdio_buffer[4096];
uint32_t lines_written = 0, foot_lines = 0;

void close_files() {
    if (log_file) fclose(log_file);
    if (foot_file) fclose(foot_file);
    log_file = foot_file = nullptr;
}

void unmount_sd() {
    if (!sd_mounted) return;
    fs.unmount();
    sd.deinit();
    sd_mounted = false;
}

bool mount_sd(bool verbose = true) {
    if (sd_mounted) return true;
    int err = sd.init();
    if (err != 0) {
        if (verbose) printf("SD init failed: %d\r\n", err);
        return false;
    }
    err = fs.mount(&sd);
    if (err != 0) {
        if (verbose) printf("SD mount failed: %d (請確認為 FAT32)\r\n", err);
        sd.deinit();
        return false;
    }
    sd_mounted = true;
    return true;
}

// 沒在紀錄時每秒檢查 SD 卡還在不在 (拔掉 → 卸載；插回 → 重新掛載)
bool check_sd() {
    if (sd_mounted) {
        static uint8_t blk[512];
        if (sd.read(blk, 0, 512) == 0) return true;  // 直接讀卡 (不經過 FAT 快取)，卡拔掉會失敗
        unmount_sd();
    }
    return mount_sd(false);
}

int cur_run = 0;

bool open_log_file() {
    if (!mount_sd()) return false;
    mkdir("/fs/exoskeleton_project_record", 0777);  // 已存在時失敗可忽略
    mkdir(LOG_DIR, 0777);

    struct stat st;
    int run = 0;
    for (int i = 1; i <= 999; i++) {
        snprintf(log_path, sizeof(log_path), "%s/run%03d.csv", LOG_DIR, i);
        if (stat(log_path, &st) != 0) {
            run = i;
            break;
        }
    }
    if (!run) {
        printf("找不到可用檔名 run001~run999.csv\r\n");
        return false;
    }
    cur_run = run;
    snprintf(foot_path, sizeof(foot_path), "%s/run%03d_foot.csv", LOG_DIR, run);
    log_file = fopen(log_path, "w");
    foot_file = log_file ? fopen(foot_path, "w") : nullptr;
    if (!log_file || !foot_file) {
        printf("無法建立 %s (errno=%d)\r\n", log_file ? foot_path : log_path, errno);
        close_files();
        return false;
    }
    setvbuf(log_file, sd_stdio_buffer, _IOFBF, sizeof(sd_stdio_buffer));
    setvbuf(foot_file, foot_stdio_buffer, _IOFBF, sizeof(foot_stdio_buffer));
    fprintf(log_file,
            "cycle,phase,t_s,trig,enc_t_s,enc1_raw,enc1_deg,enc1_stat,enc1_err,enc2_raw,enc2_deg,enc2_stat,enc2_err,"
            "imu_t_s,imu_new,imu_bad,imu_pc,imu_stf,roll,pitch,yaw,gx,gy,gz,ax,ay,az,");
    for (int k = 0; k < N_UIMU; k++) {
        char c = UIMU_NAME[k];
        fprintf(log_file,
                "imu%c_t_s,imu%c_new,imu%c_bad,imu%c_pc,imu%c_stf,imu%c_roll,imu%c_pitch,imu%c_yaw,"
                "imu%c_gx,imu%c_gy,imu%c_gz,imu%c_ax,imu%c_ay,imu%c_az,",
                c, c, c, c, c, c, c, c, c, c, c, c, c, c);
    }
    fprintf(log_file, "footL_cnt,footR_cnt,busy_us,dropped\n");
    fprintf(foot_file, "seq,foot,phase,t_rx_s,cycle,esp_us,esp_seq");
    for (int i = 1; i <= FOOT_N; i++) fprintf(foot_file, ",P%d", i);
    fprintf(foot_file, ",total_g,cop_valid,cop_x_mm,cop_y_mm\n");
    fflush(log_file);
    fflush(foot_file);
    lines_written = foot_lines = 0;
    printf("REC start: %s + run%03d_foot.csv\r\n", log_path, run);
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
        fflush(foot_file);
        close_files();
        printf("REC stop: %s, %lu lines, foot %lu lines (檔案已關閉，可以拔卡)\r\n", log_path,
               (unsigned long)lines_written, (unsigned long)foot_lines);
    }
    // SD 保持掛載，才能馬上判斷「準備好」；檔案都關了，直接拔卡不會壞檔 (拔掉會被 check_sd 偵測到)
}

bool sync_file(FILE* f) { return fflush(f) == 0 && fsync(fileno(f)) == 0 && !ferror(f); }

// fflush 只把資料交給檔案系統；fsync 才會更新 FAT 與檔案大小，斷電後檔案才讀得到
bool sync_recording() {
    if (!log_file) return true;
    if (!sync_file(log_file) || !sync_file(foot_file)) {
        printf("SD write error (errno=%d)，停止紀錄\r\n", errno);
        close_files();
        unmount_sd();
        sd_error = true;
        return false;
    }
    return true;
}

// µs → "秒.微秒"
void fprint_t(FILE* f, uint64_t t_us) {
    fprintf(f, "%lu.%06lu", (unsigned long)(t_us / 1000000), (unsigned long)(t_us % 1000000));
}

// 一顆 IMU 的欄位：,t_s,new,bad,pc,stf,roll,pitch,yaw,gx,gy,gz,ax,ay,az
void write_imu(FILE* f, uint64_t t_us, const ImuData& m) {
    fprintf(f, ",");
    fprint_t(f, t_us);
    fprintf(f, ",%u,%u,%u,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f", m.n_new, m.bad, m.pc,
            (unsigned long)m.stf, m.euler[0], m.euler[1], m.euler[2], m.gyr[0], m.gyr[1], m.gyr[2], m.acc[0],
            m.acc[1], m.acc[2]);
}

void write_sample(FILE* f, const Sample& s) {
    fprintf(f, "%lu,%u,", (unsigned long)s.cycle, s.phase);
    fprint_t(f, s.t_us);
    fprintf(f, ",%u,", s.trig);
    fprint_t(f, s.enc_t_us);
    for (int i = 0; i < N_ENC; i++) {
        fprintf(f, ",%u,%.2f,%02X,%u", s.enc[i].raw, s.enc[i].deg, s.enc[i].stat, s.enc[i].err);
    }
    write_imu(f, s.imu_t_us, s.imu);
    for (int k = 0; k < N_UIMU; k++) write_imu(f, s.uimu_t_us[k], s.uimu[k]);
    fprintf(f, ",%lu,%lu,%u,%lu\n", (unsigned long)s.foot_cnt[0], (unsigned long)s.foot_cnt[1], s.busy,
            (unsigned long)s.dropped);
}

// 足底：總重 + COP (算法同 Foot_sensor/main.cpp；總重小於門檻時 COP 留空)
void write_foot(FILE* f, const FootFrame& fr, int phase) {
    fprintf(f, "%lu,%c,%d,", (unsigned long)fr.seq, fr.foot == 0 ? 'L' : 'R', phase);
    fprint_t(f, fr.t_rx_us);
    fprintf(f, ",%lu,", (unsigned long)fr.cycle);
    if (fr.has_esp) fprintf(f, "%lu,%u", (unsigned long)fr.esp_us, fr.esp_seq);
    else fprintf(f, ",");
    float sum = 0, sx = 0, sy = 0;
    for (int i = 0; i < FOOT_N; i++) {
        fprintf(f, ",%u", fr.p[i]);
        sum += fr.p[i];
        sx += fr.p[i] * (FOOT_X[i] - FOOT_ORIGIN_X);
        sy += fr.p[i] * (FOOT_Y[i] - FOOT_ORIGIN_Y);
    }
    if (sum >= FOOT_FORCE_THRESHOLD) fprintf(f, ",%lu,1,%.2f,%.2f\n", (unsigned long)sum, sx / sum, sy / sum);
    else fprintf(f, ",%lu,0,,\n", (unsigned long)sum);
}

// ---------------- 原點 (按下開始後的 3 秒) ----------------
// 每個通道累加相對第一筆的差值 (角度處理 ±180 繞回)，結束時算平均 / 範圍，存 runXXX_origin.csv
struct OriginAcc {
    float ref, sum, mn, mx;
    uint32_t n;
    uint64_t t_first, t_last;
    void add(float v, uint64_t t, bool wrap) {
        if (n == 0) {
            ref = v;
            sum = mn = mx = 0;
            t_first = t;
        }
        float d = v - ref;
        if (wrap) d = remainderf(d, 360.0f);  // -180 ~ +180
        sum += d;
        if (d < mn) mn = d;
        if (d > mx) mx = d;
        n++;
        t_last = t;
    }
    float mean(bool wrap) const {
        float m = ref + sum / n;
        return wrap ? remainderf(m, 360.0f) : m;
    }
};

// 通道順序：enc1、enc2、3 顆 IMU x 9 (roll pitch yaw gx gy gz ax ay az)、左右腳 x (P1..P18 + total)
const int OR_ENC = 0, OR_IMU = N_ENC, OR_FOOT = OR_IMU + 3 * 9, OR_N = OR_FOOT + 2 * (FOOT_N + 1);
OriginAcc origin_acc[OR_N];
const char* IMU_FIELD[9] = {"roll", "pitch", "yaw", "gx", "gy", "gz", "ax", "ay", "az"};
const char* IMU_PREFIX[3] = {"imu", "imuA", "imuB"};

void origin_reset() { memset(origin_acc, 0, sizeof(origin_acc)); }

void origin_add_imu(int k, const ImuData& m, uint64_t t) {
    if (m.n_new == 0) return;  // 只算新封包，不重複算同一筆
    const float* v[3] = {m.euler, m.gyr, m.acc};
    for (int j = 0; j < 9; j++) origin_acc[OR_IMU + 9 * k + j].add(v[j / 3][j % 3], t, j < 3);
}

void origin_add_sample(const Sample& s) {
    for (int i = 0; i < N_ENC; i++)
        if (!(s.enc[i].err & 0x27)) origin_acc[OR_ENC + i].add(s.enc[i].deg, s.enc_t_us, true);
    origin_add_imu(0, s.imu, s.imu_t_us);
    for (int k = 0; k < N_UIMU; k++) origin_add_imu(1 + k, s.uimu[k], s.uimu_t_us[k]);
}

void origin_add_foot(const FootFrame& fr) {
    int base = OR_FOOT + fr.foot * (FOOT_N + 1);
    float sum = 0;
    for (int i = 0; i < FOOT_N; i++) {
        origin_acc[base + i].add(fr.p[i], fr.t_rx_us, false);
        sum += fr.p[i];
    }
    origin_acc[base + FOOT_N].add(sum, fr.t_rx_us, false);
}

void origin_channel_name(int c, char* name, int len) {
    if (c < OR_IMU) snprintf(name, len, "enc%d_deg", c + 1);
    else if (c < OR_FOOT) snprintf(name, len, "%s_%s", IMU_PREFIX[(c - OR_IMU) / 9], IMU_FIELD[(c - OR_IMU) % 9]);
    else {
        int f = (c - OR_FOOT) / (FOOT_N + 1), i = (c - OR_FOOT) % (FOOT_N + 1);
        if (i < FOOT_N) snprintf(name, len, "foot%c_P%d", f ? 'R' : 'L', i + 1);
        else snprintf(name, len, "foot%c_total_g", f ? 'R' : 'L');
    }
}

bool origin_is_angle(int c) { return c < OR_IMU || (c < OR_FOOT && (c - OR_IMU) % 9 < 3); }

// 寫 runXXX_origin.csv，並在序列埠印出摘要；回傳原點期間角度最大變化 (deg)
float write_origin() {
    char path[80];
    snprintf(path, sizeof(path), "%s/run%03d_origin.csv", LOG_DIR, cur_run);
    FILE* f = fopen(path, "w");
    if (f) fprintf(f, "channel,mean,min,max,n,t_first_s,t_last_s\n");
    float move_max = 0;
    char worst[24] = "-";
    for (int c = 0; c < OR_N; c++) {
        const OriginAcc& a = origin_acc[c];
        char name[24];
        origin_channel_name(c, name, sizeof(name));
        bool ang = origin_is_angle(c);
        if (a.n && ang && a.mx - a.mn > move_max) {
            move_max = a.mx - a.mn;
            strcpy(worst, name);
        }
        if (!f) continue;
        if (a.n == 0) {
            fprintf(f, "%s,,,,0,,\n", name);  // 這個裝置原點期間沒有資料
            continue;
        }
        float lo = a.ref + a.mn, hi = a.ref + a.mx;
        if (ang) {
            lo = remainderf(lo, 360.0f);
            hi = remainderf(hi, 360.0f);
        }
        fprintf(f, "%s,%.4f,%.4f,%.4f,%lu,", name, a.mean(ang), lo, hi, (unsigned long)a.n);
        fprint_t(f, a.t_first);
        fprintf(f, ",");
        fprint_t(f, a.t_last);
        fprintf(f, "\n");
    }
    bool ok = f && fclose(f) == 0;
    printf("ORIGIN: %s %s\r\n", path, ok ? "saved" : "寫入失敗");
    printf("ORIGIN: enc1=%.2f enc2=%.2f deg | imu R/P/Y=%.2f/%.2f/%.2f | A=%.2f/%.2f/%.2f | B=%.2f/%.2f/%.2f | "
           "footL=%.0fg footR=%.0fg\r\n",
           origin_acc[OR_ENC].n ? origin_acc[OR_ENC].mean(true) : NAN,
           origin_acc[OR_ENC + 1].n ? origin_acc[OR_ENC + 1].mean(true) : NAN,
           origin_acc[OR_IMU].mean(true), origin_acc[OR_IMU + 1].mean(true), origin_acc[OR_IMU + 2].mean(true),
           origin_acc[OR_IMU + 9].mean(true), origin_acc[OR_IMU + 10].mean(true), origin_acc[OR_IMU + 11].mean(true),
           origin_acc[OR_IMU + 18].mean(true), origin_acc[OR_IMU + 19].mean(true), origin_acc[OR_IMU + 20].mean(true),
           origin_acc[OR_FOOT + FOOT_N].n ? origin_acc[OR_FOOT + FOOT_N].mean(false) : 0.0f,
           origin_acc[OR_FOOT + 2 * FOOT_N + 1].n ? origin_acc[OR_FOOT + 2 * FOOT_N + 1].mean(false) : 0.0f);
    if (move_max > ORIGIN_MOVE_DEG)
        printf("WARN: 原點期間 %s 變化 %.2f deg (> %.1f)，可能有動到，建議停止後重來\r\n", worst, move_max,
               ORIGIN_MOVE_DEG);
    return move_max;
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


enum RunState { ST_IDLE, ST_ORIGIN, ST_MEASURE };

int main() {
    spi_enc.format(8, 2);  // AS5145：CLK 平時 High，下降緣取樣 (同 encoder/main.cpp 的 Mode 2)
    spi_enc.frequency(ENC_SPI_FREQ);
    spi_imu.format(8, 3);
    spi_imu.frequency(IMU_SPI_FREQ);
    user_button.mode(PullUp);
    clock_us.start();
    led = 0;

    printf("\r\n==== 2x AS5145 + SPI IMU + 2x UART IMU + foot sensor -> microSD @100Hz ====\r\n");
    foot_uart.attach(callback(on_foot_rx), SerialBase::RxIrq);  // 足底 UART 一開機就開始收
    for (int k = 0; k < N_UIMU; k++) uimu[k].start();
    ThisThread::sleep_for(3000ms);  // 等 IMU 開機
    for (int k = 0; k < N_UIMU; k++) uimu[k].send_cmd(MID_GOTOMEAS);  // UART IMU 停在 Config 模式時叫它開始輸出

    init_imu();
    load_zero();
    print_encoder_check();

    if (imu_ok) {
        use_drdy = true;
        imu_drdy.rise(callback(on_drdy));
    } else {
        printf("WARN: SPI IMU 未初始化，使用 100Hz ticker\r\n");
        use_ticker();
    }
    sample_thread.start(sample_loop);
    mount_sd();  // 放在取樣開始之後：SD 沒插或初始化很慢時，感測器照常讀取

    FileHandle* console = mbed_file_handle(STDIN_FILENO);
    int live_div = 0;  // 即時輸出：0 = 關、10 = 每 10 個週期一筆 (10Hz)、2 = 50Hz

    // 狀態：IDLE (沒在紀錄；準備好 = LD2 恆亮) → 按 B1 → ORIGIN (3 秒) → MEASURE → 按 B1 → IDLE
    RunState st = ST_IDLE;
    bool ready = false;
    uint64_t origin_t0 = 0;
    bool origin_seen_meas = false;  // 已經收到 phase = 2 的週期 (原點的週期都處理完了)

    auto start_run = [&]() {
        if (st != ST_IDLE) return;
        if (!start_recording()) {
            printf("REC: SD 開檔失敗，無法開始\r\n");
            return;
        }
        origin_reset();
        origin_seen_meas = false;
        origin_t0 = now_us();
        origin_end_us = origin_t0 + ORIGIN_US;
        rec_on = true;
        st = ST_ORIGIN;
        printf("ORIGIN: 記錄原點 %d 秒，請保持不動...\r\n", (int)(ORIGIN_US / 1000000));
    };
    auto stop_run = [&]() {
        if (st == ST_IDLE) return;
        rec_on = false;
        st = ST_IDLE;
        stop_recording();
    };

    // encoder 歸零 (存 flash)
    int zero_n = 0;                 // > 0：歸零中，還要收集幾筆
    int32_t zero_sum[N_ENC] = {};   // 相對第一筆的差值總和 (處理 4095→0 跨越)
    uint16_t zero_ref[N_ENC] = {};
    int zero_got = 0;
    auto request_zero = [&]() {
        if (st != ST_IDLE) {
            printf("ZERO: 紀錄中不能歸零 (寫 flash 會讓取樣停 1~2 秒)，先停止紀錄\r\n");
            return;
        }
        zero_n = 50;
        zero_got = 0;
        printf("ZERO: 收集 50 筆中，請保持不動...\r\n");
    };

    bool button_prev = false;
    uint32_t wr_sum_us = 0, wr_max_us = 0;
    uint64_t button_t = 0, last_status = now_us(), last_led = 0;
    uint32_t n_sec = 0, enc_err[N_ENC] = {}, enc_warn[N_ENC] = {}, imu_miss = 0, imu_bad = 0, trig_drdy = 0;
    uint16_t busy_max = 0;
    uint32_t uimu_prev_ok[N_UIMU] = {}, uimu_prev_bad[N_UIMU] = {}, uimu_prev_bytes[N_UIMU] = {};
    // 足底每秒統計：收到幾包、最大間隔、ESP32→STM32 延遲的抖動 (max - min)
    uint32_t foot_n[2] = {}, foot_gap_max[2] = {}, foot_esp_n = 0;
    uint64_t foot_last_t[2] = {};
    uint32_t lat_ref = 0;
    int32_t lat_min = 0, lat_max = 0;

    while (true) {
        // 足底封包：寫 runXXX_foot.csv + 原點統計 + 每秒統計
        while (FootFrame* fr = foot_mail.try_get()) {
            if (st != ST_IDLE && fr->t_rx_us >= origin_t0) {
                int phase = fr->t_rx_us < origin_end_us ? 1 : 2;
                if (foot_file) {
                    write_foot(foot_file, *fr, phase);
                    foot_lines++;
                }
                if (st == ST_ORIGIN && phase == 1) origin_add_foot(*fr);
            }
            int k = fr->foot;
            foot_n[k]++;
            if (foot_last_t[k]) {
                uint32_t gap = (uint32_t)(fr->t_rx_us - foot_last_t[k]);
                if (gap > foot_gap_max[k]) foot_gap_max[k] = gap;
            }
            foot_last_t[k] = fr->t_rx_us;
            if (fr->has_esp) {
                uint32_t d = (uint32_t)fr->t_rx_us - fr->esp_us;  // MCU 時鐘 - ESP32 時鐘 (常數 + 延遲)
                if (foot_esp_n == 0) lat_ref = d;
                int32_t r = (int32_t)(d - lat_ref);
                if (foot_esp_n == 0 || r < lat_min) lat_min = r;
                if (foot_esp_n == 0 || r > lat_max) lat_max = r;
                foot_esp_n++;
            }
            foot_mail.free(fr);
        }

        Sample* s = mail.try_get_for(5ms);
        if (s) {
            if (log_file && s->phase != 0) {  // 按下開始之前還在佇列裡的週期 (phase 0) 不寫
                uint64_t w0 = now_us();
                write_sample(log_file, *s);
                uint32_t w = (uint32_t)(now_us() - w0);
                if (w > wr_max_us) wr_max_us = w;
                wr_sum_us += w;
                lines_written++;
            }
            if (st == ST_ORIGIN) {
                if (s->phase == 1) origin_add_sample(*s);
                else if (s->phase == 2) origin_seen_meas = true;
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
                    // 抹除 flash 時 CPU 會停住；先停掉 1kHz Ticker / DRDY / UART 中斷，不然 RTOS 的 ISR 佇列會塞爆當機
                    sample_ticker.detach();
                    imu_drdy.rise(nullptr);
                    foot_uart.attach(nullptr, SerialBase::RxIrq);  // 這段時間的足底 / UART IMU 封包會遺失
                    for (int k = 0; k < N_UIMU; k++) uimu[k].stop();
                    bool ok = save_zero(z);
                    for (int k = 0; k < N_UIMU; k++) uimu[k].start();
                    foot_uart.attach(callback(on_foot_rx), SerialBase::RxIrq);
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
            core_util_atomic_decr_u32(&q_depth, 1);
        }

        uint64_t now = now_us();

        // 原點 3 秒結束：原點的週期都處理完 (已經收到 phase 2) 且足底封包也到齊 (多等 100ms) → 存原點、開始測量
        if (st == ST_ORIGIN && origin_seen_meas && now >= origin_end_us + 100000) {
            write_origin();
            st = ST_MEASURE;
            printf("MEAS: 開始測量 (再按一次 B1 停止)\r\n");
        }
        // 紀錄中 SD 出錯 (sync_recording 已關檔)：回到 IDLE
        if (st != ST_IDLE && !log_file) {
            rec_on = false;
            st = ST_IDLE;
            printf("REC: SD 錯誤，紀錄已停止\r\n");
        }

        // 每秒：寫回 SD + 印狀態 + 檢查是否準備好
        if (now - last_status >= 1000000) {
            last_status = now;
            uint64_t sy0 = now_us();
            sync_recording();
            uint32_t sync_us = (uint32_t)(now_us() - sy0);
            // 寫檔診斷：wr = 這一秒寫 CSV 花的總時間 / 單行最久；sync = fflush+fsync；q_max = 佇列最多積幾筆 (200 滿)
            if (log_file)
                printf("SD wr_sum=%lums wr_max=%lums sync=%lums q_max=%lu\r\n", (unsigned long)(wr_sum_us / 1000),
                       (unsigned long)(wr_max_us / 1000), (unsigned long)(sync_us / 1000), (unsigned long)q_max);
            wr_sum_us = wr_max_us = 0;
            q_max = q_depth;
            uint32_t urate[N_UIMU], ubad[N_UIMU], ubytes[N_UIMU];
            for (int k = 0; k < N_UIMU; k++) {
                uint32_t ok = uimu[k].pkt_ok, bad = uimu[k].pkt_bad, by = uimu[k].rx_bytes;
                urate[k] = ok - uimu_prev_ok[k];
                ubad[k] = bad - uimu_prev_bad[k];
                ubytes[k] = by - uimu_prev_bytes[k];
                uimu_prev_ok[k] = ok;
                uimu_prev_bad[k] = bad;
                uimu_prev_bytes[k] = by;
            }
            printf("STAT rec=%d lines=%lu rate=%lu imu_trig=%lu enc_err=%lu/%lu mag_warn=%lu/%lu imu_miss=%lu imu_bad=%lu busy_max=%uus "
                   "dropped=%lu\r\n",
                   log_file != nullptr, (unsigned long)lines_written, (unsigned long)n_sec,
                   (unsigned long)trig_drdy, (unsigned long)enc_err[0], (unsigned long)enc_err[1], (unsigned long)enc_warn[0],
                   (unsigned long)enc_warn[1],
                   (unsigned long)imu_miss, (unsigned long)imu_bad, busy_max, (unsigned long)dropped);
            // 每秒封包數 / 收到 byte 數 / 錯誤封包數。0 B/s = 沒接好 (TX/RX 接反、GND、供電)；
            // 有 byte 但 0 包 = 鮑率不對 (UIMU_BAUD 跟 IMU 設定不同)
            printf("UIMU A=%lu/s %luB/s (bad %lu) B=%lu/s %luB/s (bad %lu)", (unsigned long)urate[0],
                   (unsigned long)ubytes[0], (unsigned long)ubad[0], (unsigned long)urate[1], (unsigned long)ubytes[1],
                   (unsigned long)ubad[1]);
            for (int k = 0; k < N_UIMU; k++) {
                if (urate[k] >= (uint32_t)READY_MIN_RATE) continue;
                if (ubytes[k] == 0) printf("  %c:沒收到任何byte(檢查接線)", UIMU_NAME[k]);
                else if (urate[k] == 0) printf("  %c:有byte沒封包(鮑率%d不對?)", UIMU_NAME[k], UIMU_BAUD[k]);
            }
            printf("\r\n");
            // footL/R = 這一秒收到幾包 (最大間隔 ms)；foot_bad = checksum 錯；foot_jit = ESP32→STM32 延遲抖動 (µs)
            printf("FOOT L=%lu (gap_max=%lums) R=%lu (gap_max=%lums) total L/R=%lu/%lu foot_bad=%lu foot_drop=%lu ",
                   (unsigned long)foot_n[0], (unsigned long)(foot_gap_max[0] / 1000), (unsigned long)foot_n[1],
                   (unsigned long)(foot_gap_max[1] / 1000), (unsigned long)foot_cnt[0], (unsigned long)foot_cnt[1],
                   (unsigned long)foot_bad, (unsigned long)foot_drop);
            if (foot_esp_n) printf("foot_jit=%ldus\r\n", (long)(lat_max - lat_min));
            else printf("foot_jit=- (ESP32 沒送時間戳)\r\n");

            if (st == ST_IDLE) {
                bool sd_ok = check_sd();
                if (sd_ok) sd_error = false;
                bool enc_ok = n_sec >= (uint32_t)READY_MIN_RATE && enc_err[0] + enc_err[1] <= n_sec / 20;
                bool spi_ok = imu_ok && n_sec - imu_miss >= (uint32_t)READY_MIN_RATE;
                bool u_ok[N_UIMU];
                for (int k = 0; k < N_UIMU; k++) u_ok[k] = urate[k] >= (uint32_t)READY_MIN_RATE;
                bool foot_ok = foot_n[0] > 0 && foot_n[1] > 0;
                ready = sd_ok && (!REQ_ENC || enc_ok) && (!REQ_SPI_IMU || spi_ok) &&
                        (!REQ_UART_IMU || (u_ok[0] && u_ok[1])) && (!REQ_FOOT || foot_ok);
                auto tag = [](bool ok, bool req) { return ok ? "OK" : (req ? "NG" : "-"); };
                printf("READY=%d  enc=%s spiIMU=%s imuA=%s imuB=%s foot=%s SD=%s%s\r\n", ready, tag(enc_ok, REQ_ENC),
                       tag(spi_ok, REQ_SPI_IMU), tag(u_ok[0], REQ_UART_IMU), tag(u_ok[1], REQ_UART_IMU),
                       tag(foot_ok, REQ_FOOT), tag(sd_ok, true), ready ? "  -> 按 B1 開始 (先記錄 3 秒原點)" : "");
            }

            n_sec = trig_drdy = imu_miss = imu_bad = busy_max = 0;
            for (int i = 0; i < N_ENC; i++) enc_err[i] = enc_warn[i] = 0;
            foot_n[0] = foot_n[1] = foot_gap_max[0] = foot_gap_max[1] = foot_esp_n = 0;
        }

        // B1 (50ms 消抖)，放開時動作：IDLE 且準備好 → 開始 (原點 3 秒 → 測量)；紀錄中 → 停止
        bool pressed = user_button.read() == 0;
        if (pressed != button_prev && now - button_t > 50000) {
            button_prev = pressed;
            button_t = now;
            if (!pressed) {
                if (st != ST_IDLE) stop_run();
                else if (ready) start_run();
                else printf("還沒準備好 (LD2 沒亮)，看 READY 那一行哪一項是 NG；要強制開始請在序列埠送 'g'\r\n");
            }
        }

        if (console->readable()) {
            char ch;
            if (console->read(&ch, 1) == 1) {
                if (ch == 's' || ch == 'S') {
                    int div = ch == 's' ? 10 : 2;
                    live_div = live_div == div ? 0 : div;
                }
                else if (ch == 'g') start_run();
                else if (ch == 'x') stop_run();
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

        // LD2：沒準備好 = 熄滅、準備好 = 恆亮、原點 3 秒 = 快閃、測量中 = 慢閃
        if (st == ST_ORIGIN || st == ST_MEASURE) {
            uint64_t half = st == ST_ORIGIN ? 100000 : 500000;
            if (now - last_led >= half) {
                led = !led;
                last_led = now;
            }
        } else {
            led = ready ? 1 : 0;
        }
    }
}
