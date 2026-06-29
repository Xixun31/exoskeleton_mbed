#include "mbed.h"
#include "MTi2/MTi2.h"
#include <cstring>

static BufferedSerial pc(USBTX, USBRX, 115200);

// ================= SPI 設定 =================
// Frequency: 1MHz, MOSI: PB_15, MISO: PB_14, SCLK: PB_13, CS: PB_1
MTi2Class IMU(200000, PB_15, PB_14, PB_13, PB_1);
DigitalOut led(LED1);

int main() {
    char buffer[256];
    int len = snprintf(buffer, sizeof(buffer), "--- STM32 IMU Reader (SPI Mode) ---\n");
    pc.write(buffer, len);

    // 初始化 IMU (設定 DRDY 等)
    IMU.MTi2_Init();
    thread_sleep_for(100); // 提供 100ms 穩定延遲

    while (true) {
        // 從 SPI 讀取資料
        IMU.ReadData();

        // 格式化輸出以與 python/stm32_imu_uart.py 相容：
        // 歐拉角 (deg), 加速度 (m/s^2), 角速度 (deg/s)
        // 註：IMU.omega 原本為 rad/s，乘上 57.2958f 轉成 deg/s
        /*printf("R:%.2f P:%.2f Y:%.2f | aX:%.2f aY:%.2f aZ:%.2f | gX:%.2f gY:%.2f gZ:%.2f\n",
            IMU.euler[0], IMU.euler[1], IMU.euler[2],
            IMU.accel[0], IMU.accel[1], IMU.accel[2],
            IMU.omega[0] * 57.2958f, IMU.omega[1] * 57.2958f, IMU.omega[2] * 57.2958f);*/
        /*// 把所有的數值強制加上 (int) 轉成整數，避開 Mbed OS 的浮點數限制
        pc.printf("R:%d P:%d Y:%d | aX:%d aY:%d aZ:%d | gX:%d gY:%d gZ:%d\n",
               (int)IMU.euler[0], (int)IMU.euler[1], (int)IMU.euler[2],
               (int)IMU.accel[0], (int)IMU.accel[1], (int)IMU.accel[2],
               (int)(IMU.omega[0] * 57.2958f), (int)(IMU.omega[1] * 57.2958f), (int)(IMU.omega[2] * 57.2958f));*/
        // 在 main.cpp 的 while 迴圈內
        
        uint32_t euler0, euler1, euler2;
        uint32_t accel0, accel1, accel2;
        uint32_t omega0, omega1, omega2;

        memcpy(&euler0, &IMU.euler[0], sizeof(euler0));
        memcpy(&euler1, &IMU.euler[1], sizeof(euler1));
        memcpy(&euler2, &IMU.euler[2], sizeof(euler2));
        memcpy(&accel0, &IMU.accel[0], sizeof(accel0));
        memcpy(&accel1, &IMU.accel[1], sizeof(accel1));
        memcpy(&accel2, &IMU.accel[2], sizeof(accel2));
        memcpy(&omega0, &IMU.omega[0], sizeof(omega0));
        memcpy(&omega1, &IMU.omega[1], sizeof(omega1));
        memcpy(&omega2, &IMU.omega[2], sizeof(omega2));

        // 必須嚴格保持這個格式，確保每個 Hex 數值前面都有冒號，且總共有 9 個數值
        len = snprintf(buffer, sizeof(buffer),
            "R:%08lX P:%08lX Y:%08lX A:%08lX A:%08lX A:%08lX G:%08lX G:%08lX G:%08lX\n",
            (unsigned long)euler0, (unsigned long)euler1, (unsigned long)euler2,
            (unsigned long)accel0, (unsigned long)accel1, (unsigned long)accel2,
            (unsigned long)omega0, (unsigned long)omega1, (unsigned long)omega2);
        pc.write(buffer, len);

        // 閃爍 LED1 指示運行狀態
        led = !led;

        // 延遲 50ms (20Hz 更新率)
        thread_sleep_for(50);
    }
}