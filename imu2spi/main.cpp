#include "mbed.h"
#include "MTi2.h"

// ================= 獨立雙 SPI 設定 =================
// IMU1 使用 SPI3 (全接 Arduino 母孔，不需與其他訊號線共用插孔)
// MOSI: PB_5 (D4), MISO: PB_4 (D5), SCLK: PB_3 (D3), CS: PB_6 (D10)
SPI spi_bus3(PB_5, PB_4, PB_3);
MTi2Class IMU1(spi_bus3, PB_6);

// IMU2 使用 SPI2 (全接 Morpho 公排針，與 IMU1 獨立，不需共用腳位)
// MOSI: PB_15 (CN10 Pin 26), MISO: PB_14 (CN10 Pin 28), SCLK: PB_13 (CN10 Pin 30), CS: PB_1 (CN10 Pin 24)
SPI spi_bus2(PB_15, PB_14, PB_13);
MTi2Class IMU2(spi_bus2, PB_1);

int main() {
    printf("--- STM32 Dual IMU Reader (Independent SPIs: SPI3 & SPI2) ---\n");

    // 初始化兩顆 IMU
    IMU1.MTi2_Init();
    IMU2.MTi2_Init();
    thread_sleep_for(100); // 穩定延遲

    while (true) {
        // 1. 讀取第一顆 IMU
        IMU1.ReadData();
        
        // 給予 SPI 匯流排極短的切換延遲以保證訊號穩定
        thread_sleep_for(2);
        
        // 2. 讀取第二顆 IMU
        IMU2.ReadData();

        // 格式化輸出 18 個數值（IMU1 的 9 個 + IMU2 的 9 個）：
        // 格式：R1 P1 Y1 aX1 aY1 aZ1 gX1 gY1 gZ1 | R2 P2 Y2 aX2 aY2 aZ2 gX2 gY2 gZ2
        // 註：IMU.omega 原本為 rad/s，乘上 57.2958f 轉成 deg/s
        printf("R1:%6.2f P1:%6.2f Y1:%6.2f | aX1:%5.2f aY1:%5.2f aZ1:%5.2f | gX1:%6.2f gY1:%6.2f gZ1:%6.2f | "
               "R2:%6.2f P2:%6.2f Y2:%6.2f | aX2:%5.2f aY2:%5.2f aZ2:%5.2f | gX2:%6.2f gY2:%6.2f gZ2:%6.2f\n",
               IMU1.euler[0], IMU1.euler[1], IMU1.euler[2],
               IMU1.accel[0], IMU1.accel[1], IMU1.accel[2],
               IMU1.omega[0] * 57.2958f, IMU1.omega[1] * 57.2958f, IMU1.omega[2] * 57.2958f,
               IMU2.euler[0], IMU2.euler[1], IMU2.euler[2],
               IMU2.accel[0], IMU2.accel[1], IMU2.accel[2],
               IMU2.omega[0] * 57.2958f, IMU2.omega[1] * 57.2958f, IMU2.omega[2] * 57.2958f);

        // 50ms 週期 (20Hz 更新率)
        thread_sleep_for(50);
    }
}
