#include "mbed.h"
#include "MTi2.h"

// ================= SPI 設定 =================
// Frequency: 1MHz, MOSI: PB_15, MISO: PB_14, SCLK: PB_13, CS: PB_1
MTi2Class IMU(200000, PB_15, PB_14, PB_13, PB_1);
DigitalOut led(LED1);

int main() {
    printf("--- STM32 IMU Reader (SPI Mode) ---\n");

    // 初始化 IMU (設定 DRDY 等)
    IMU.MTi2_Init();
    thread_sleep_for(100); // 提供 100ms 穩定延遲

    while (true) {
        // 從 SPI 讀取資料
        IMU.ReadData();

        // 格式化輸出以與 python/stm32_imu_uart.py 相容：
        // 歐拉角 (deg), 加速度 (m/s^2), 角速度 (deg/s)
        // 註：IMU.omega 原本為 rad/s，乘上 57.2958f 轉成 deg/s
        printf("R:%6.2f P:%6.2f Y:%6.2f | aX:%5.2f aY:%5.2f aZ:%5.2f | gX:%6.2f gY:%6.2f gZ:%6.2f\n",
               IMU.euler[0], IMU.euler[1], IMU.euler[2],
               IMU.accel[0], IMU.accel[1], IMU.accel[2],
               IMU.omega[0] * 57.2958f, IMU.omega[1] * 57.2958f, IMU.omega[2] * 57.2958f);

        // 閃爍 LED1 指示運行狀態
        led = !led;

        // 延遲 50ms (20Hz 更新率)
        thread_sleep_for(50);
    }
}
