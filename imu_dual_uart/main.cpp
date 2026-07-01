#include "mbed.h"

// ================= IMU 類別宣告與實作 =================
class IMUReader {
public:
    float euler[3]; 
    float accel[3]; 
    float omega[3]; 
    volatile uint32_t total_rx_bytes;
    volatile bool new_data_flag;

    IMUReader(PinName tx, PinName rx, int baud = 115200) 
        : serial(tx, rx, baud), total_rx_bytes(0), new_data_flag(false),
          rx_state(0), expected_length(0), payload_idx(0) {
        memset(euler, 0, sizeof(euler));
        memset(accel, 0, sizeof(accel));
        memset(omega, 0, sizeof(omega));
    }

    void init() {
        serial.attach(callback(this, &IMUReader::rx_isr), SerialBase::RxIrq);
    }

private:
    UnbufferedSerial serial;
    uint8_t rx_state;
    uint8_t expected_length;
    uint8_t payload_buffer[256];
    uint8_t payload_idx;

    float swap_float(const uint8_t* data) {
        uint32_t temp = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
        float result;
        memcpy(&result, &temp, 4);
        return result;
    }

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

    void rx_isr() {
        char c;
        if (serial.read(&c, 1)) {
            uint8_t byte = (uint8_t)c;
            total_rx_bytes++;       

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
                        new_data_flag = true;
                        rx_state = 0; 
                    }
                    break;
                default: rx_state = 0; break;
            }
        }
    }
};

// ================= 全局變數 =================
DigitalOut led(LED1);

// ================= 主程式 =================
int main() {
    printf("\n--- STM32 Dual IMU UART Reader ---\n");

    // 建立兩個 IMU 物件，分別指定接腳
    IMUReader imu1(PA_9, PA_10, 115200);   // IMU 1 接腳 (例如 UART1)
    IMUReader imu2(PC_10, PC_11, 115200); // IMU 2 接腳 (例如 UART4 或 USART3)

    // 初始化接收中斷
    imu1.init();
    imu2.init();

    while (true) {
        thread_sleep_for(50); // 50ms 更新率 (20Hz)

        // 若任一 IMU 有收到新資料，閃爍 LED
        if (imu1.new_data_flag || imu2.new_data_flag) {
            led = !led;
            imu1.new_data_flag = false;
            imu2.new_data_flag = false;
        }

        // 輸出 IMU 1 資料
        printf("IMU1 -> R:%6.2f P:%6.2f Y:%6.2f | aX:%5.2f aY:%5.2f aZ:%5.2f | gX:%6.2f gY:%6.2f gZ:%6.2f\n",
               imu1.euler[0], imu1.euler[1], imu1.euler[2],
               imu1.accel[0], imu1.accel[1], imu1.accel[2],
               imu1.omega[0] * 57.2958f, imu1.omega[1] * 57.2958f, imu1.omega[2] * 57.2958f);

        // 輸出 IMU 2 資料
        printf("IMU2 -> R:%6.2f P:%6.2f Y:%6.2f | aX:%5.2f aY:%5.2f aZ:%5.2f | gX:%6.2f gY:%6.2f gZ:%6.2f\n",
               imu2.euler[0], imu2.euler[1], imu2.euler[2],
               imu2.accel[0], imu2.accel[1], imu2.accel[2],
               imu2.omega[0] * 57.2958f, imu2.omega[1] * 57.2958f, imu2.omega[2] * 57.2958f);
               
        printf("------------------------------------------------------------------------------------\n");
    }
}
