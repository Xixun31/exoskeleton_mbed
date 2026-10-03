// =====================================================================
// IMU vs Absolute Encoder Frequency Response Firmware
//
// Based on motor_control/main.cpp (same commands: sine / feedback / reset / <deg>).
// Differences:
//   - Control loop runs at an exact 100Hz (sleep_until instead of sleep_for)
//   - Encoder and IMU are sampled together in the control loop and
//     timestamped on the board (us), so USB latency does not affect phase
//   - Encoder reads with bit15 != 1 are rejected (previous value is kept)
//   - Output line (100Hz):
//     D,t_us,enc_deg,roll,pitch,yaw,target_deg,inc_deg,imu_packets,enc_errors,dropped,imu_age_us,gx,gy,gz
//     imu_age_us = how long ago the IMU packet used in this sample arrived
//     gx,gy,gz   = IMU rate of turn (deg/s), from the same packet as roll/pitch/yaw
// =====================================================================
#include "mbed.h"
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// --- Motor parameters ---
const float RR = 30.0f; // Reduction ratio of the motor
// Motor wiring direction: +1 or -1. Flip this if the motor runs away (spins forever)
// because the motor turns opposite to the incremental encoder count.
const float MOTOR_DIR = -1.0f;
const float RAD2DEG = 180.0f / 3.14159265358979323846f;

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
    volatile uint32_t packet_count; // Number of parsed MTData2 packets (to estimate IMU rate)
    volatile uint32_t last_packet_us; // us_ticker time when the last packet was parsed

    IMUReader(PinName tx, PinName rx, int baud = 115200)
        : serial(tx, rx, baud), new_data_flag(false), packet_count(0), last_packet_us(0),
          rx_state(0), expected_length(0), payload_idx(0) {
        memset(euler, 0, sizeof(euler));
        memset(accel, 0, sizeof(accel));
        memset(omega, 0, sizeof(omega));
    }

    void init() {
        serial.attach(callback(this, &IMUReader::rx_isr), SerialBase::RxIrq);
    }

    // Copy euler angles atomically (the RX ISR may update them at any time)
    void get_euler(float out[3], float out_omega[3], uint32_t* out_packets, uint32_t* out_age_us) {
        core_util_critical_section_enter();
        memcpy(out, euler, sizeof(euler));
        memcpy(out_omega, omega, sizeof(omega));
        *out_packets = packet_count;
        *out_age_us = us_ticker_read() - last_packet_us;
        core_util_critical_section_exit();
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
                        packet_count++;
                        last_packet_us = us_ticker_read();
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

// PC commands are received in the RX interrupt so no byte is lost while
// the main thread is busy writing log lines
CircularBuffer<char, 128> cmd_rx_buf;
Mutex pc_tx_mutex; // main thread and command thread both write to pc

void pc_rx_isr() {
    char c;
    while (pc.readable()) {
        if (pc.read(&c, 1) > 0) {
            cmd_rx_buf.push(c);
        }
    }
}

void pc_write_locked(const char* buf, int len) {
    pc_tx_mutex.lock();
    pc.write(buf, len);
    pc_tx_mutex.unlock();
}

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

// --- Synchronized Log Samples (control thread -> main thread) ---
struct LogSample {
    uint64_t t_us;       // Board timestamp when encoder was read
    float enc_deg;       // Absolute encoder (0 ~ 360)
    float euler[3];      // IMU roll, pitch, yaw (deg)
    float omega[3];      // IMU rate of turn x, y, z (rad/s)
    float target_deg;    // Commanded angle
    float inc_deg;       // Incremental encoder angle
    uint32_t imu_packets;
    uint32_t imu_age_us; // Age of the IMU data at the moment of sampling
};
Mail<LogSample, 32> log_mail;
Timer log_timer;
volatile uint32_t enc_error_count = 0; // Encoder reads rejected (bit15 != 1)
volatile uint32_t log_dropped = 0;     // Samples lost because the mailbox was full

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
    printf("IMU vs Encoder Frequency Response Firmware Started!\r\n");
    printf("Format: D,t_us,enc_deg,roll,pitch,yaw,target_deg,inc_deg,imu_packets,enc_errors,dropped,imu_age_us,gx,gy,gz\r\n");
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
    pc.attach(callback(pc_rx_isr), SerialBase::RxIrq);
    cmd_thread.start(callback(serial_cmd_loop));

    // Attach button interrupts
    button.fall(&button_pressed_isr);
    button.rise(&button_released_isr);

    // Start 100Hz control loop thread
    log_timer.start();
    control_thread.start(callback(control_thread_loop));

    // Main thread: print every sample produced by the control loop
    // (encoder and IMU were captured at the same instant, with a board timestamp)
    while (true) {
        LogSample* s = log_mail.try_get_for(Kernel::wait_for_u32_forever);
        if (s == nullptr) {
            continue;
        }

        char tx_buf[128];
        int len = sprintf(tx_buf, "D,%llu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu,%lu,%lu,%lu,%.1f,%.1f,%.1f\r\n",
                          s->t_us, s->enc_deg, s->euler[0], s->euler[1], s->euler[2],
                          s->target_deg, s->inc_deg, (unsigned long)s->imu_packets,
                          (unsigned long)enc_error_count, (unsigned long)log_dropped,
                          (unsigned long)s->imu_age_us,
                          s->omega[0] * RAD2DEG, s->omega[1] * RAD2DEG, s->omega[2] * RAD2DEG);
        log_mail.free(s);
        pc_write_locked(tx_buf, len);
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

    // Bit15 is always 1 in a valid frame; otherwise keep the previous reading
    static uint16_t last_valid = 0;
    uint16_t valid_data;
    if (raw_data & 0x8000) {
        // Parse valid data: D11 ~ D0
        valid_data = (raw_data >> 3) & 0x0FFF;
        last_valid = valid_data;
    } else {
        enc_error_count++;
        valid_data = last_valid;
    }

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
    uint64_t t_us = log_timer.elapsed_time().count();
    abs_raw_val = read_absolute_encoder(&abs_deg_val_temp);
    abs_degree_val = abs_deg_val_temp;

    // Capture IMU at the same instant as the encoder
    float euler_snapshot[3];
    float omega_snapshot[3];
    uint32_t imu_packets = 0;
    uint32_t imu_age_us = 0;
    imu.get_euler(euler_snapshot, omega_snapshot, &imu_packets, &imu_age_us);

    int32_t counter = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    float inc_angle = -(float)counter / (48.0f * RR) * 2.0f * M_PI;

    if (feedback_source == 0) {
        current_angle = inc_angle;
    } else {
        float relative_deg = angular_difference_mbed(abs_degree_val, abs_baseline_deg);
        current_angle = relative_deg * (M_PI / 180.0f);
    }

    LogSample* s = log_mail.try_alloc();
    if (s != nullptr) {
        s->t_us = t_us;
        s->enc_deg = abs_deg_val_temp;
        memcpy(s->euler, euler_snapshot, sizeof(euler_snapshot));
        memcpy(s->omega, omega_snapshot, sizeof(omega_snapshot));
        s->target_deg = target_angle * 180.0f / M_PI;
        s->inc_deg = inc_angle * 180.0f / M_PI;
        s->imu_packets = imu_packets;
        s->imu_age_us = imu_age_us;
        log_mail.put(s);
    } else {
        log_dropped++;
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

    float duty_cycle = (MOTOR_DIR * voltage + 12.0f) / 24.0f * 100.0f;

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

// --- Serial Command Parsing Loop (runs in thread, bytes come from pc_rx_isr) ---
void serial_cmd_loop() {
    char local_buf[32];
    int idx = 0;
    char ch;
    while (true) {
        if (cmd_rx_buf.empty()) {
            ThisThread::sleep_for(2ms);
            continue;
        }
        if (cmd_rx_buf.pop(ch)) {
            if (ch == '\r' || ch == '\n') {
                local_buf[idx] = '\0';
                if (idx > 0) {
                    char debug_buf[64];
                    int debug_len = sprintf(debug_buf, ">> CMD:[%s] len=%d\r\n", local_buf, idx);
                    pc_write_locked(debug_buf, debug_len);

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
    auto next_tick = Kernel::Clock::now();
    while (true) {
        next_tick += 10ms;
        control_loop_100hz();
        ThisThread::sleep_until(next_tick); // Fixed 10ms period, no drift
    }
}
