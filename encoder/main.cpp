#include "mbed.h"

// --- Encoder SPI 設定 ---
SPI spi(D4, D5, D3);       // mosi, miso, sclk
DigitalOut encoder_cs(D9); // SPI_CS1

// 變數
uint16_t raw_data = 0;
uint16_t valid_data = 0;
float degree = 0.0f;

// --- 解析度設定 ---
// 根據你的編碼器規格修改。這裡以 14-bit 為例 (2^14 = 16384)
// 若為 12-bit，請改成 4096.0f
const float ENCODER_RESOLUTION = 4096.0f;

// 函式宣告
void init_IO();
void init_SPI();
void read_position();
void print_angle();

int main() {
  printf("Single Encoder SPI Reader Started!\r\n");

  init_IO();
  init_SPI();

  while (true) {
      thread_sleep_for(50); // 每 50 毫秒讀取一次
      read_position();
      print_angle(); 
  }
}

void init_IO() { 
  encoder_cs = 1; 
}

void init_SPI() {
  spi.format(16, 2);      // 16-bit 資料長度, SPI Mode 3
  spi.frequency(1000000); // 1MHz clock rate
}

void read_position() {
  encoder_cs = 0; // 拉低 CS，開始與編碼器通訊
  wait_us(10);    // 給予 IC 短暫的反應時間

  // 讀取前 16-bit 的資料 
  raw_data = spi.write(0x0000);

  encoder_cs = 1; // 拉高 CS，結束通訊

  // --- 資料解析與角度換算 ---
  // 1. 向右位移 3 格，讓 D0 對齊最低位元
  // 2. 加上 & 0x0FFF (二進位的 0000_1111_1111_1111) 把最前面的恆定 1 給濾掉
  valid_data = (raw_data >> 3) & 0x0FFF;

  // 換算成 0 ~ 360 度
  degree = ((float)valid_data / ENCODER_RESOLUTION) * 360.0f;
}

void print_angle() {
    // 同時印出「過濾後的原始數值」以及「換算後的角度」，方便你除錯
    printf("%d, %.2f\r\n", valid_data, degree);
}