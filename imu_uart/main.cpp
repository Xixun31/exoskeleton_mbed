#include "mbed.h"

// ================= UART 設定 =================
UnbufferedSerial imu_serial(PA_9, PA_10, 115200); 
DigitalOut led(LED1);

volatile uint32_t total_rx_bytes = 0;   
volatile uint8_t last_rx_byte = 0;      

// ================= IMU 變數與解析 =================
float euler[3] = {0}; 
float accel[3] = {0}; 
float omega[3] = {0}; 

float swap_float(const uint8_t* data) {
    uint32_t temp = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    float result;
    memcpy(&result, &temp, 4);
    return result;
}

// 加入終極安全機制的解析器
void parse_mtdata2(uint8_t* data, uint8_t length) {
    uint8_t i = 0;
    // 安全網 1：確保至少有 3 個 byte 可以讀 (XDI 2 byte + Size 1 byte)
    while (i + 2 < length) {
        uint16_t xdi = (data[i] << 8) | data[i+1];
        uint8_t size = data[i+2];
        
        // 安全網 2：如果宣告的資料大小 超出了我們實際收到的長度 -> 絕對是亂碼，立刻停止解析！
        if (i + 3 + size > length) {
            break; // 逃生出口，避免記憶體越界崩潰 (Hard Fault)
        }

        uint8_t* content = &data[i+3];

        if (xdi == 0x2030 && size == 12) {        
            euler[0] = swap_float(content);     
            euler[1] = swap_float(content + 4); 
            euler[2] = swap_float(content + 8); 
        }
        else if (xdi == 0x4020 && size == 12) {   
            accel[0] = swap_float(content);
            accel[1] = swap_float(content + 4);
            accel[2] = swap_float(content + 8);
        }
        else if (xdi == 0x8020 && size == 12) {   
            omega[0] = swap_float(content);
            omega[1] = swap_float(content + 4);
            omega[2] = swap_float(content + 8);
        }
        i += (3 + size);
    }
}

// ================= UART 接收中斷狀態機 =================
uint8_t rx_state = 0;
uint8_t expected_length = 0;
uint8_t payload_buffer[256];
uint8_t payload_idx = 0;

void imu_rx_isr() {
    char c;
    // 拔掉 while，改成 if：每次中斷只處理一個字元，把 CPU 控制權還給主迴圈
    if (imu_serial.read(&c, 1)) {
        uint8_t byte = (uint8_t)c;
        
        total_rx_bytes++;       
        last_rx_byte = byte;    

        switch (rx_state) {
            case 0: if (byte == 0xFA) rx_state = 1; break;
            case 1: if (byte == 0xFF) rx_state = 2; else rx_state = 0; break;
            case 2: if (byte == 0x36) rx_state = 3; else rx_state = 0; break;
            case 3: 
                expected_length = byte;
                payload_idx = 0;
                // 安全網 3：防止長度異常大
                if (expected_length > 0 && expected_length < 250) rx_state = 4;
                else rx_state = 0; 
                break;
            case 4: 
                if (payload_idx < 255) { 
                    payload_buffer[payload_idx++] = byte;
                }
                if (payload_idx > expected_length) { 
                    parse_mtdata2(payload_buffer, expected_length);
                    led = !led;   
                    rx_state = 0; 
                }
                break;
            default: rx_state = 0; break;
        }
    }
}

// ================= 主程式 =================
int main() {
    printf("--- STM32 IMU Reader (Full Sensor Mode) ---\n");

    imu_serial.attach(&imu_rx_isr, SerialBase::RxIrq);

    while (true) {
        // 恢復 50ms 更新率 (20Hz)
        thread_sleep_for(50); 
        
        // 解除封印：印出小數點，並加入加速度與角速度
        // omega 乘上 57.2958f 轉成 deg/s
        printf("R:%6.2f P:%6.2f Y:%6.2f | aX:%5.2f aY:%5.2f aZ:%5.2f | gX:%6.2f gY:%6.2f gZ:%6.2f\n",
               euler[0], euler[1], euler[2],
               accel[0], accel[1], accel[2],
               omega[0] * 57.2958f, omega[1] * 57.2958f, omega[2] * 57.2958f);
    }
}