#include "mbed.h"
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// --- Motor parameters ---
const float RR = 30.0f; // Reduction ratio of the motor

// --- Hardware Timer Handles ---
TIM_HandleTypeDef htim1; // TIM1 for PWM
TIM_HandleTypeDef htim2; // TIM2 for Encoder

// --- Thread for 100Hz Control Loop ---
Thread control_thread(osPriorityAboveNormal, 4096);

// --- InterruptIn for Blue Button ---
InterruptIn button(PC_13);

// --- IMU UART Reader Class ---
class IMUReader {
public:
    float euler[3]; 
    float accel[3]; 
    float omega[3]; 
    volatile bool new_data_flag;

    IMUReader(PinName tx, PinName rx, int baud = 115200) 
        : serial(tx, rx, baud), new_data_flag(false),
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
        while (i + 2 < length) {
            uint16_t xdi = (data[i] << 8) | data[i+1];
            uint8_t size = data[i+2];
            if (i + 3 + size > length) {
                break;
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
            switch (rx_state) {
                case 0: if (byte == 0xFA) rx_state = 1; break;
                case 1: if (byte == 0xFF) rx_state = 2; else rx_state = 0; break;
                case 2: if (byte == 0x36) rx_state = 3; else rx_state = 0; break;
                case 3: 
                    expected_length = byte;
                    payload_idx = 0;
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

IMUReader imu(PC_10, PC_11, 115200);
UnbufferedSerial pc(USBTX, USBRX, 115200);

// --- Global variables for control ---
volatile float target_angle = 0.0f;  // Unit: rad
volatile float current_angle = 0.0f; // Unit: rad
volatile float prev_angle = 0.0f;    // Unit: rad
volatile float velocity = 0.0f;     // Unit: rad/s
volatile float voltage = 0.0f;      // Unit: V
volatile float pos_error = 0.0f;     // Unit: rad
volatile float prev_pos_error = 0.0f;// Unit: rad
volatile float integral = 0.0f;     // Unit: rad * s
const float Ts = 0.01f;              // Control period (100Hz = 0.01s)

// --- PID Controller Parameters ---
const float Kp = 3.1f;
const float Ki = 0.5f;
const float Kd = 0.15f;
const float V_dead = 7.0f; 

// Button state tracking
volatile bool button_pressed = false;
volatile int toggle_count = 0;

// --- SPI Absolute Encoder ---
SPI spi_enc(D4, D5, D3);       // MOSI, MISO, SCLK
DigitalOut encoder_cs(D9);     // Chip Select 1 (Output verification / Shaft)

volatile uint16_t abs_raw_val = 0;
volatile float abs_degree_val = 0.0f;
volatile float abs_baseline_deg = 0.0f;

// Feedback source: 0 = Incremental (TIM2), 1 = Absolute Encoder (D9)
volatile int feedback_source = 0;

// --- Sinusoidal Sweep Mode Variables ---
volatile bool sine_mode = false;
volatile float sine_amplitude = 0.0f;
volatile float sine_frequency = 0.0f;
volatile float sine_offset = 0.0f;
volatile uint32_t sine_step_count = 0;

// Thread for serial command parsing
Thread cmd_thread;

// --- Function Declarations ---
void init_encoder();
void init_pwm();
void init_spi_encoder();
uint16_t read_absolute_encoder(float* out_deg);
float angular_difference_mbed(float a, float b);
void serial_cmd_loop();
void control_loop_100hz();
void control_thread_loop();
void button_pressed_isr();
void button_released_isr();

int main() {
    // Wait for system to stabilize and serial port to connect
    ThisThread::sleep_for(1s);
    printf("\r\n==================================================\r\n");
    printf("Mbed OS 6 Motor Position Control System Started!\r\n");
    printf("Kp: %.3f, Ki: %.3f, Kd: %.3f\r\n", Kp, Ki, Kd);
    printf("==================================================\r\n");

    // Initialize hardware peripherals
    init_encoder();
    init_pwm();
    init_spi_encoder();
    imu.init(); // Initialize IMU UART Rx interrupt

    // Read baseline absolute encoder angle for zero-calibration reference
    read_absolute_encoder((float*)&abs_baseline_deg);

    // Start serial command listener thread (blocking-based, completely safe)
    cmd_thread.start(callback(serial_cmd_loop));

    // Attach button interrupts
    button.fall(&button_pressed_isr);
    button.rise(&button_released_isr);

    // Start 100Hz control loop thread
    control_thread.start(callback(control_thread_loop));

    // Main thread loop for debugging printouts
    while (true) {
        // Read absolute encoders (already populated by 100Hz control ticker)

        // Read IMU angles safely (already in degrees from Xsens)
        float imu_roll = imu.euler[0];
        float imu_pitch = imu.euler[1];

        // Read PID values safely
        float t_ang = target_angle;
        float c_ang = current_angle;
        float volt = voltage;

        // Convert angles to degrees for output
        float target_deg = t_ang * 180.0f / M_PI;
        float current_deg = c_ang * 180.0f / M_PI;

        // Print format for automated calibration & IMU testing:
        // absolute raw, absolute deg, imu roll, imu pitch, voltage, target deg, incremental deg
        char tx_buf[128];
        int len = sprintf(tx_buf, "%d, %.2f, %.2f, %.2f, %.4f, %.4f, %.4f\r\n", 
                          abs_raw_val, abs_degree_val, imu_roll, imu_pitch, volt, target_deg, current_deg);
        pc.write(tx_buf, len);

        // Print frequency of 100Hz (10ms)
        ThisThread::sleep_for(10ms);
    }
}

// --- Initialize Encoder TIM2 ---
void init_encoder() {
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295U;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef sConfig = {0};
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
        printf("Error: TIM2 Encoder Init Failed!\r\n");
    }
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
        printf("Error: TIM2 Encoder Start Failed!\r\n");
    }
}

// --- Initialize PWM TIM1 ---
void init_pwm() {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 18 - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 100 - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        printf("Error: TIM1 Base Init Failed!\r\n");
    }

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

    HAL_TIM_PWM_Init(&htim1);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}

// --- Initialize SPI Absolute Encoder ---
void init_spi_encoder() {
    encoder_cs = 1;
    spi_enc.format(16, 2);      // 16-bit, SPI Mode 2
    spi_enc.frequency(1000000); // 1MHz clock frequency
}

// --- Read SPI Absolute Encoder ---
uint16_t read_absolute_encoder(float* out_deg) {
    encoder_cs = 0; // Pull CS low to select the encoder
    wait_us(10);
    
    // Write dummy data to read 16-bit word
    uint16_t raw_data = spi_enc.write(0x0000);
    
    encoder_cs = 1; // Pull CS high to end communication
    
    // Parse valid data: D11 ~ D0
    uint16_t valid_data = (raw_data >> 3) & 0x0FFF;
    
    if (out_deg) {
        // Convert 12-bit value to degrees (0 to 360)
        *out_deg = ((float)valid_data / 4096.0f) * 360.0f;
    }
    
    return valid_data;
}

// --- Angular Difference Helper ---
float angular_difference_mbed(float a, float b) {
    float diff = fmodf(a - b + 180.0f, 360.0f);
    if (diff < 0.0f) diff += 360.0f;
    return diff - 180.0f;
}

// --- 100Hz Control Loop Callback ---
void control_loop_100hz() {
    if (sine_mode) {
        float t = (float)sine_step_count * Ts;
        target_angle = sine_offset + sine_amplitude * sinf(2.0f * M_PI * sine_frequency * t);
        sine_step_count++;
    }

    // Read absolute encoder safely (only here to prevent SPI bus collision)
    float abs_deg_val_temp = 0.0f;
    abs_raw_val = read_absolute_encoder(&abs_deg_val_temp);
    abs_degree_val = abs_deg_val_temp;

    if (feedback_source == 0) {
        int32_t counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        current_angle = -(float)counter / (48.0f * RR) * 2.0f * M_PI;
    } else {
        float relative_deg = angular_difference_mbed(abs_degree_val, abs_baseline_deg);
        current_angle = relative_deg * (M_PI / 180.0f);
    }

    velocity = (current_angle - prev_angle) / Ts;
    prev_angle = current_angle;

    pos_error = target_angle - current_angle;

    if (fabsf(pos_error) < 0.005f) {
        voltage = 0.0f;
        integral = 0.0f;
        prev_pos_error = pos_error;
    } else {
        float u_p = Kp * pos_error;

        integral += pos_error * Ts;
        float u_i = Ki * integral;

        float derivative = (pos_error - prev_pos_error) / Ts;
        prev_pos_error = pos_error;
        float u_d = Kd * derivative;

        float u_raw = u_p + u_i + u_d;

        float u_comp = 0.0f;
        if (u_raw > 0.05f) {
            u_comp = u_raw + V_dead;
        } else if (u_raw < -0.05f) {
            u_comp = u_raw - V_dead;
        } else {
            u_comp = 0.0f;
        }

        if (u_comp > 12.0f) {
            u_comp = 12.0f;
            integral -= pos_error * Ts; // Prevent windup
        } else if (u_comp < -12.0f) {
            u_comp = -12.0f;
            integral -= pos_error * Ts; // Prevent windup
        }

        voltage = u_comp;
    }

    float duty_cycle = (voltage + 12.0f) / 24.0f * 100.0f;

    if (duty_cycle > 100.0f) duty_cycle = 100.0f;
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)duty_cycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)duty_cycle);
}

// --- Button ISR (Pressed) ---
void button_pressed_isr() {
    if (!button_pressed) {
        button_pressed = true;
        toggle_count++;
        if (toggle_count % 2 == 1) {
            target_angle = M_PI / 2.0f;  // +90 deg
        } else {
            target_angle = -M_PI / 2.0f; // -90 deg
        }
    }
}

// --- Button ISR (Released) ---
void button_released_isr() {
    button_pressed = false;
}

// --- Blocking Serial Command Parsing Loop (runs in thread) ---
void serial_cmd_loop() {
    char local_buf[32];
    int idx = 0;
    char ch;
    while (true) {
        if (pc.read(&ch, 1) > 0) {
            if (ch == '\r' || ch == '\n') {
                local_buf[idx] = '\0';
                if (idx > 0) {
                    char debug_buf[64];
                    int debug_len = sprintf(debug_buf, ">> CMD:[%s] len=%d\r\n", local_buf, idx);
                    pc.write(debug_buf, debug_len);

                    if (strncmp(local_buf, "sine ", 5) == 0) {
                        float amp_deg = 0.0f;
                        float freq_hz = 0.0f;
                        float offset_deg = 0.0f;
                        int parsed = sscanf(local_buf + 5, "%f %f %f", &amp_deg, &freq_hz, &offset_deg);
                        if (parsed >= 2) {
                            sine_amplitude = amp_deg * M_PI / 180.0f;
                            sine_frequency = freq_hz;
                            if (parsed == 3) {
                                sine_offset = offset_deg * M_PI / 180.0f;
                            } else {
                                sine_offset = 0.0f;
                            }
                            sine_step_count = 0;
                            sine_mode = true;
                        }
                    } else if (strncmp(local_buf, "feedback ", 9) == 0) {
                        int mode = atoi(local_buf + 9);
                        if (mode >= 0 && mode <= 1) {
                            feedback_source = mode;
                            // Reset PID variables to prevent jumps
                            integral = 0.0f;
                            prev_pos_error = 0.0f;
                        }
                    } else if (strncmp(local_buf, "reset", 5) == 0) {
                        __HAL_TIM_SET_COUNTER(&htim2, 0);
                        integral = 0.0f;
                        prev_pos_error = 0.0f;
                        target_angle = 0.0f;
                        sine_mode = false;
                    } else {
                        sine_mode = false;
                        float target_deg = atof(local_buf);
                        target_angle = target_deg * M_PI / 180.0f;
                    }
                }
                idx = 0;
            } else if (ch == 8 || ch == 127) {
                if (idx > 0) idx--;
            } else if (idx < 31 && ch >= ' ' && ch <= '~') {
                local_buf[idx++] = ch;
            }
        }
    }
}

// --- Thread Loop for 100Hz Control ---
void control_thread_loop() {
    while (true) {
        control_loop_100hz();
        ThisThread::sleep_for(10ms); // Sleep for 10ms to achieve 100Hz loop rate
    }
}
