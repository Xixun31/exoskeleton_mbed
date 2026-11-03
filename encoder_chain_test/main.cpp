// =====================================================================
// 2 顆 AS5145 daisy chain 串接測試 (接線同 enc_imu_sd_logger)
//   SPI1：SCK PB_3 (CN10-31)、MISO PB_4 (CN10-27)、MOSI PB_5 (不用接)、CS PC_7 (CN10-19)
//   CLK、CSn 兩顆並接；encoder1 DO → MISO；encoder2 DO → encoder1 PROG；encoder2 PROG → GND
//
// 開機後自動執行：
//   1. RAW：連讀 64 bit 印成二進位，依 [起始1 | enc1 18bit | dummy | enc2 18bit | 其餘] 分段
//   2. STATIC：100Hz 讀 1000 次 (保持靜止)，統計每顆的錯誤次數與讀值跳動
//   3. LIVE：10Hz 連續輸出角度 (轉動確認哪顆是 enc1 / enc2)
// 指令：'r' RAW、's' STATIC、'l' LIVE 開關、'1'/'2'/'3' SPI clock = 100k / 500k / 1M、'p'/'n' MISO 上拉 開/關、'o' 舊讀法對照、'h'/'H' CS 固定 Low/High (電表量測)
//
// 每顆 18 bit = D11..D0, OCF, COF, LIN, MagINC, MagDEC, Parity(偶同位)
// err：bit0 同位錯、bit1 OCF=0、bit2 COF=1、bit3 LIN=1、bit4 磁場超出範圍、bit5 起始 bit 不是 1
// =====================================================================
#include "mbed.h"

SPI spi_enc(PB_5, PB_4, PB_3);  // mosi, miso, sclk
DigitalOut enc_cs(PC_7, 1);
int spi_freq = 1000000;
const int N_ENC = 2;

struct EncData {
    uint16_t raw;
    uint8_t stat, err;
};

// 讀 8 bytes，回傳 64 bit (bit63 = 第一個 bit)
uint64_t read_bits() {
    char tx[8] = {}, b[8];
    enc_cs = 0;
    wait_us(1);
    spi_enc.write(tx, 8, b, 8);
    enc_cs = 1;
    wait_us(1);
    uint64_t v = 0;
    for (int k = 0; k < 8; k++) v = (v << 8) | (uint8_t)b[k];
    return v;
}

// 第 i 顆的 18 bit 從第 (1 + 19i) 個 bit 開始
EncData parse(uint64_t v, int i) {
    uint32_t w = (v >> (64 - 1 - 19 * i - 18)) & 0x3FFFF;
    EncData e;
    e.raw = w >> 6;
    e.stat = w & 0x3F;
    e.err = 0;
    if (__builtin_parity(w)) e.err |= 0x01;
    if (!(w & 0x20)) e.err |= 0x02;
    if (w & 0x10) e.err |= 0x04;
    if (w & 0x08) e.err |= 0x08;
    if ((w & 0x04) && (w & 0x02)) e.err |= 0x10;
    if (!(v >> 63)) e.err |= 0x20;
    return e;
}

float to_deg(uint16_t raw) { return raw * (360.0f / 4096.0f); }

void print_bits(uint64_t v, int from, int n) {
    for (int k = from; k < from + n; k++) putchar((v >> (63 - k)) & 1 ? '1' : '0');
}

void raw_dump() {
    printf("RAW_START,spi=%d\r\n", spi_freq);
    printf("     s|enc1 D11..D0  OCLMMP|d|enc2 D11..D0  OCLMMP|rest\r\n");
    for (int n = 0; n < 10; n++) {
        uint64_t v = read_bits();
        printf("RAW, ");
        print_bits(v, 0, 1);
        putchar('|');
        print_bits(v, 1, 12);
        putchar(' ');
        print_bits(v, 13, 6);
        putchar('|');
        print_bits(v, 19, 1);
        putchar('|');
        print_bits(v, 20, 12);
        putchar(' ');
        print_bits(v, 32, 6);
        putchar('|');
        print_bits(v, 38, 26);
        for (int i = 0; i < N_ENC; i++) {
            EncData e = parse(v, i);
            printf(",enc%d=%u/%.2f/err%02X", i + 1, e.raw, to_deg(e.raw), e.err);
        }
        printf("\r\n");
        ThisThread::sleep_for(100ms);
    }
    printf("RAW_END\r\n");
}

// 通訊錯誤 (同位 / OCF / COF / 起始 bit) 的讀值不採用；磁場警告 (LIN、MagINC+MagDEC) 只計數，讀值照樣統計
const uint8_t COMM_ERR = 0x01 | 0x02 | 0x04 | 0x20;

void static_test(int n) {
    printf("STATIC_START,n=%d,spi=%d (保持靜止)\r\n", n, spi_freq);
    int comm[N_ENC] = {}, err_bits[N_ENC][6] = {}, good[N_ENC] = {};
    int vmin[N_ENC], vmax[N_ENC];
    double sum[N_ENC] = {};
    for (int i = 0; i < N_ENC; i++) {
        vmin[i] = 4096;
        vmax[i] = -1;
    }
    auto next = Kernel::Clock::now();
    for (int k = 0; k < n; k++) {
        uint64_t v = read_bits();
        for (int i = 0; i < N_ENC; i++) {
            EncData e = parse(v, i);
            for (int b = 0; b < 6; b++) err_bits[i][b] += (e.err >> b) & 1;
            if (e.err & COMM_ERR) {
                comm[i]++;
                continue;
            }
            good[i]++;
            sum[i] += e.raw;
            if (e.raw < vmin[i]) vmin[i] = e.raw;
            if (e.raw > vmax[i]) vmax[i] = e.raw;
        }
        next += 10ms;
        ThisThread::sleep_until(next);
    }
    for (int i = 0; i < N_ENC; i++) {
        double mean = good[i] ? sum[i] / good[i] : 0;
        printf("STATIC,enc%d,comm_err=%d/%d,par=%d,ocf=%d,cof=%d,start=%d | warn lin=%d,mag=%d | min=%d,max=%d,span=%d,mean=%.2f(%.2fdeg)\r\n",
               i + 1, comm[i], n, err_bits[i][0], err_bits[i][1], err_bits[i][2], err_bits[i][5], err_bits[i][3],
               err_bits[i][4], good[i] ? vmin[i] : 0, good[i] ? vmax[i] : 0, good[i] ? vmax[i] - vmin[i] : 0, mean,
               to_deg((uint16_t)mean));
    }
    printf("STATIC_END\r\n");
}

// 對照：用 encoder_speed_test 的讀法 (16-bit 單筆，wait 1us)，確認 8-bit 多 byte 讀法沒問題
void old_style_read() {
    spi_enc.format(16, 2);
    printf("OLD_START (16-bit, bit15 應恆為 1)\r\n");
    int ok = 0;
    for (int n = 0; n < 20; n++) {
        enc_cs = 0;
        wait_us(1);
        uint16_t a = spi_enc.write(0x0000);
        uint16_t b = spi_enc.write(0x0000);
        uint16_t c = spi_enc.write(0x0000);
        enc_cs = 1;
        ok += (a >> 15) & 1;
        printf("OLD,%04X %04X %04X,deg=%.2f\r\n", a, b, c, to_deg((a >> 3) & 0x0FFF));
        ThisThread::sleep_for(10ms);
    }
    printf("OLD_END,bit15_ok=%d/20\r\n", ok);
    spi_enc.format(8, 2);
}

int main() {
    spi_enc.format(8, 2);  // CLK 平時 High，下降緣取樣
    spi_enc.frequency(spi_freq);
    printf("\r\n==== AS5145 daisy chain test ====\r\n");
    ThisThread::sleep_for(100ms);

    raw_dump();
    static_test(1000);
    bool live = true;
    printf("LIVE (轉動 encoder 確認順序；'l' 開關)\r\n");

    FileHandle* console = mbed_file_handle(STDIN_FILENO);
    auto next = Kernel::Clock::now();
    while (true) {
        char ch;
        if (console->readable() && console->read(&ch, 1) == 1) {
            if (ch == 'r') raw_dump();
            else if (ch == 's') static_test(1000);
            else if (ch == 'l') live = !live;
            else if (ch == 'o') old_style_read();
            else if (ch == 'h' || ch == 'H') {
                // 三用電表量測用：停止讀取並把 CS 固定在 Low ('h') 或 High ('H')，CLK 保持 High；'l' 恢復讀取
                live = false;
                enc_cs = ch == 'h' ? 0 : 1;
                printf("HOLD: CS = %s\r\n", ch == 'h' ? "LOW" : "HIGH");
            }
            else if (ch == 'p' || ch == 'n') {
                // 診斷用：MISO 加內部上拉。DO 沒在輸出 (浮接) 時會讀到全 1，被拉 Low 則仍是 0
                pin_mode(PB_4, ch == 'p' ? PullUp : PullNone);
                printf("MISO pull-up = %d\r\n", ch == 'p');
            }
            else if (ch >= '1' && ch <= '3') {
                const int freqs[] = {100000, 500000, 1000000};
                spi_freq = freqs[ch - '1'];
                spi_enc.frequency(spi_freq);
                printf("SPI clock = %d Hz\r\n", spi_freq);
            }
            next = Kernel::Clock::now();
        }
        if (live) {
            uint64_t v = read_bits();
            EncData a = parse(v, 0), b = parse(v, 1);
            printf("LIVE,enc1=%7.2f,err%02X,enc2=%7.2f,err%02X,stat=%02X/%02X\r\n", to_deg(a.raw), a.err,
                   to_deg(b.raw), b.err, a.stat, b.stat);
        }
        next += 100ms;
        ThisThread::sleep_until(next);
    }
}
