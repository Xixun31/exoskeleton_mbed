#include "mbed.h"

// 宣告類比輸入腳位，對應 Nucleo 板子上的 A0 (PA_0)
AnalogIn flexSensor(PA_0);

// 宣告序列埠，設定為與 Python 腳本一致的 115200 波特率
static BufferedSerial pc(USBTX, USBRX, 115200);

int main() {
    // 用來存放格式化字串的緩衝區
    char buffer[64];

    while (true) {
        float sensor_ratio = flexSensor.read();
        float voltage = sensor_ratio * 3.3f;
        int adc_raw = (int)(sensor_ratio * 4095);

        // --- 神奇的整數拆解法開始 ---
        
        // 1. 抓出整數部分 (例如 1.63 會變成 1)
        int v_int = (int)voltage;
        
        // 2. 抓出小數部分，乘上 100 變成整數 (例如 (1.63 - 1) * 100 = 63)
        int v_dec = (int)((voltage - v_int) * 100);

        // 3. 用 %d (整數) 的方式把小數點「畫」出來！
        // %02d 代表如果小數是 5，會自動補零變成 05
        int len = sprintf(buffer, "ADC: %d | Voltage: %d.%02d V\r\n", adc_raw, v_int, v_dec);
        
        // --- 神奇的整數拆解法結束 ---

        pc.write(buffer, len);
        thread_sleep_for(50); 
    }
}