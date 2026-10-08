#include "mbed.h"

// =====================================================================
// Encoder SPI 最高讀取頻率測試
//
// 測試方式：
//   對每一組 (SPI clock, CS 拉低後等待時間) 連續讀取 N_SAMPLES 次，
//   量測總耗時 -> 換算成最高讀取頻率 (Hz)，同時檢查資料是否正確。
//
// 資料正確性判斷 (測試時請讓編碼器保持「靜止不動」)：
//   1. raw 最高位 (bit15) 應該恆為 1，不是 1 代表通訊錯誤
//   2. 角度值和參考值 (最慢、最保守設定讀到的值) 相差不能超過 TOLERANCE
//
// 結果最後會列出「0 錯誤且最快」的設定，就是可用的最高讀取頻率。
// =====================================================================

// --- Encoder SPI 設定 (與 encoder/main.cpp 相同) ---
SPI spi(D4, D5, D3);       // mosi, miso, sclk
DigitalOut encoder_cs(D9); // SPI_CS1

// --- 測試參數 ---
const int N_SAMPLES = 5000; // 每組設定讀取次數
const int TOLERANCE = 2;    // 靜止時允許的跳動 (counts)

// 要測試的 SPI clock (Hz)，請勿超過編碼器規格書上限
const int SPI_FREQS[] = {500000, 1000000, 2000000, 4000000, 8000000, 10000000};
// CS 拉低後等待的時間 (us)，原本程式用 10us
const int CS_DELAYS_US[] = {10, 5, 2, 1, 0};

const int N_FREQS = sizeof(SPI_FREQS) / sizeof(SPI_FREQS[0]);
const int N_DELAYS = sizeof(CS_DELAYS_US) / sizeof(CS_DELAYS_US[0]);

struct Result {
  int spi_freq;
  int cs_delay_us;
  float rate_hz;    // 每秒可讀取次數
  float avg_us;     // 平均每次讀取耗時
  float max_us;     // 最長單次讀取耗時
  int bad_header;   // bit15 不是 1 的次數
  int bad_value;    // 數值跳動超過 TOLERANCE 的次數
};

Result results[N_FREQS * N_DELAYS];

// 函式宣告
void init_IO();
void init_SPI(int freq);
void init_cycle_counter();
uint16_t read_raw(int cs_delay_us);
int angle_diff(int a, int b);
int get_reference();
Result run_test(int spi_freq, int cs_delay_us, int ref);
void print_result(const Result &r);

int main() {
  printf("\r\n=== Encoder SPI Max Read Rate Test ===\r\n");
  printf("請保持編碼器靜止不動！\r\n");
  printf("CPU clock: %lu Hz, samples per test: %d\r\n\r\n",
         (unsigned long)SystemCoreClock, N_SAMPLES);

  init_IO();
  init_cycle_counter();
  thread_sleep_for(500);

  int ref = get_reference();
  if (ref < 0) {
    printf("錯誤：用最保守設定 (500kHz, 10us) 也讀不到正確資料，請檢查接線。\r\n");
    while (true) {
      thread_sleep_for(1000);
    }
  }
  printf("參考值: %d (%.2f deg)\r\n\r\n", ref, ref / 4096.0f * 360.0f);

  printf("SPI_Hz    CS_us  avg_us  max_us   rate_Hz  bad_hdr  bad_val  status\r\n");
  printf("--------  -----  ------  ------  --------  -------  -------  ------\r\n");

  int count = 0;
  for (int i = 0; i < N_FREQS; i++) {
    for (int j = 0; j < N_DELAYS; j++) {
      results[count] = run_test(SPI_FREQS[i], CS_DELAYS_US[j], ref);
      print_result(results[count]);
      count++;
    }
  }

  // 找出 0 錯誤中讀取頻率最高的設定
  int best = -1;
  for (int k = 0; k < count; k++) {
    const Result &r = results[k];
    if (r.bad_header == 0 && r.bad_value == 0 &&
        (best < 0 || r.rate_hz > results[best].rate_hz)) {
      best = k;
    }
  }

  printf("\r\n=== 結論 ===\r\n");
  if (best >= 0) {
    const Result &r = results[best];
    printf("最高可用讀取頻率: %.0f Hz (平均 %.2f us/次, 最長 %.2f us)\r\n",
           r.rate_hz, r.avg_us, r.max_us);
    printf("設定: spi.frequency(%d), CS 拉低後 wait_us(%d)\r\n",
           r.spi_freq, r.cs_delay_us);
    printf("註：這是只讀 SPI 的上限，實際迴圈還要扣掉運算與 printf 時間。\r\n");
  } else {
    printf("沒有任何設定是 0 錯誤，請確認編碼器是否靜止或接線是否正確。\r\n");
  }

  while (true) {
    thread_sleep_for(1000);
  }
}

void init_IO() {
  encoder_cs = 1;
}

void init_SPI(int freq) {
  spi.format(16, 2);   // 16-bit 資料長度, SPI Mode 2 (與原程式相同)
  spi.frequency(freq);
}

// 使用 Cortex-M4 的 DWT cycle counter 量測單次讀取時間 (解析度 1/180MHz)
void init_cycle_counter() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint16_t read_raw(int cs_delay_us) {
  encoder_cs = 0;
  if (cs_delay_us > 0) {
    wait_us(cs_delay_us);
  }
  uint16_t raw = spi.write(0x0000);
  encoder_cs = 1;
  return raw;
}

// 考慮 0/4095 繞圈的角度差 (counts)
int angle_diff(int a, int b) {
  int d = (a - b) & 0x0FFF;
  return (d > 2048) ? 4096 - d : d;
}

// 用最保守的設定讀取參考值：連續 32 次都要一致才採用
int get_reference() {
  init_SPI(500000);
  for (int attempt = 0; attempt < 10; attempt++) {
    uint16_t raw = read_raw(10);
    if (!(raw & 0x8000)) {
      continue;
    }
    int ref = (raw >> 3) & 0x0FFF;
    bool stable = true;
    for (int i = 0; i < 32; i++) {
      uint16_t r = read_raw(10);
      if (!(r & 0x8000) || angle_diff((r >> 3) & 0x0FFF, ref) > TOLERANCE) {
        stable = false;
        break;
      }
      wait_us(100);
    }
    if (stable) {
      return ref;
    }
    thread_sleep_for(100);
  }
  return -1;
}

Result run_test(int spi_freq, int cs_delay_us, int ref) {
  Result r = {spi_freq, cs_delay_us, 0.0f, 0.0f, 0.0f, 0, 0};
  init_SPI(spi_freq);

  // 暖身，讓 SPI 設定生效
  for (int i = 0; i < 10; i++) {
    read_raw(cs_delay_us);
  }

  uint32_t max_cycles = 0;
  uint32_t start_total = DWT->CYCCNT;

  for (int i = 0; i < N_SAMPLES; i++) {
    uint32_t t0 = DWT->CYCCNT;
    uint16_t raw = read_raw(cs_delay_us);
    uint32_t dt = DWT->CYCCNT - t0;

    if (dt > max_cycles) {
      max_cycles = dt;
    }
    if (!(raw & 0x8000)) {
      r.bad_header++;
    } else if (angle_diff((raw >> 3) & 0x0FFF, ref) > TOLERANCE) {
      r.bad_value++;
    }
  }

  uint32_t total_cycles = DWT->CYCCNT - start_total;

  // 180MHz 下 CYCCNT 約 23 秒溢位，測試遠小於此，用 cycle 計算較精準
  float cycles_per_us = SystemCoreClock / 1000000.0f;
  float total_us = total_cycles / cycles_per_us;
  r.avg_us = total_us / N_SAMPLES;
  r.max_us = max_cycles / cycles_per_us;
  r.rate_hz = N_SAMPLES * 1000000.0f / total_us;
  return r;
}

void print_result(const Result &r) {
  const char *status = (r.bad_header == 0 && r.bad_value == 0) ? "OK" : "FAIL";
  printf("%8d  %5d  %6.2f  %6.2f  %8.0f  %7d  %7d  %s\r\n", r.spi_freq,
         r.cs_delay_us, r.avg_us, r.max_us, r.rate_hz, r.bad_header,
         r.bad_value, status);
}
