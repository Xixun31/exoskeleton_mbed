#include "mbed.h"
#include <chrono>

// ============================================================
// UART
// ESP32 GPIO17 TX -> STM32 PA10 RX
// ESP32 GND       -> STM32 GND
// PA9 是 STM32 TX，目前不用接
// ============================================================
BufferedSerial esp32(PA_9, PA_10, 115200);
BufferedSerial pc(USBTX, USBRX, 115200);

// ============================================================
// 感測器封包
// byte[0]  = 0xAA
// byte[1]  = 0x01 LEFT / 0x02 RIGHT
// byte[2]~byte[37] = 18 點壓力，每點 2 bytes (High + Low)
// byte[38] = checksum
// ============================================================
const int FRAME_SIZE = 39;
const int PRESSURE_COUNT = 18;

uint8_t frame[FRAME_SIZE];
int frame_index = 0;

// ============================================================
// 計時
// ============================================================
Timer timer;
bool timer_started = false;

long long get_elapsed_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        timer.elapsed_time()
    ).count();
}

// ============================================================
// 18 個感測區域幾何中心（AutoCAD 世界座標，mm）
// ============================================================
const float sensor_x[PRESSURE_COUNT] = {
    1943.9325f, 1940.0133f, 1973.6715f, 1972.9752f,
    1965.7372f, 1957.7049f, 1963.0637f, 1955.6792f,
    1995.8721f, 1996.5690f, 1983.1920f, 1980.9491f,
    1982.3130f, 1970.9490f, 2000.9966f, 2003.7015f,
    2001.4963f, 1988.7695f
};

const float sensor_y[PRESSURE_COUNT] = {
    2600.1604f, 2637.9524f, 2444.2083f, 2482.2769f,
    2521.7471f, 2561.9473f, 2599.0265f, 2642.2744f,
    2444.2212f, 2482.2769f, 2520.6611f, 2559.8462f,
    2599.0265f, 2641.6030f, 2521.0309f, 2560.0963f,
    2597.5737f, 2633.6218f
};

// 之後決定腳跟原點後改這裡
float origin_x = 0.0f;
float origin_y = 0.0f;

// 如果上面這組座標是「右腳」CAD，而你希望左腳用相同身體座標系鏡射，
// 把這個改成 true。若左右腳各自使用自己的局部座標系，保持 false。
const bool MIRROR_LEFT_X = false;

// 空載雜訊門檻（先暫定 50 g）
const float FORCE_THRESHOLD = 50.0f;

// ============================================================
// COP 結果
// ============================================================
struct COPResult
{
    bool valid;
    float x;
    float y;
    float total_force;
};

// ============================================================
// 每一隻腳各自保存最新一筆資料
// ============================================================
struct FootData
{
    uint16_t pressure[PRESSURE_COUNT];

    bool cop_valid;
    float cop_x;
    float cop_y;
    float total_force;

    long long timestamp_ms;
    long long last_timestamp_ms;
    long long total_interval_ms;
    long long max_interval_ms;

    unsigned long frame_count;
};

FootData left_foot = {};
FootData right_foot = {};

unsigned long total_good_frames = 0;
unsigned long bad_checksum = 0;

// ============================================================
// checksum
// ============================================================
bool check_checksum(const uint8_t *data)
{
    uint16_t sum = 0;

    for (int i = 0; i < 38; i++)
    {
        sum += data[i];
    }

    uint8_t checksum = (uint8_t)(sum & 0xFF);
    return checksum == data[38];
}

// ============================================================
// 計算 COP
// foot_id: 0x01 LEFT / 0x02 RIGHT
// ============================================================
COPResult calculate_cop(
    const uint16_t pressure[PRESSURE_COUNT],
    uint8_t foot_id
)
{
    COPResult result;
    result.valid = false;
    result.x = 0.0f;
    result.y = 0.0f;
    result.total_force = 0.0f;

    float sum_F = 0.0f;
    float sum_Fx = 0.0f;
    float sum_Fy = 0.0f;

    for (int i = 0; i < PRESSURE_COUNT; i++)
    {
        float F = (float)pressure[i];

        float x = sensor_x[i] - origin_x;
        float y = sensor_y[i] - origin_y;

        if (MIRROR_LEFT_X && foot_id == 0x01)
        {
            x = -x;
        }

        sum_F += F;
        sum_Fx += F * x;
        sum_Fy += F * y;
    }

    result.total_force = sum_F;

    if (sum_F < FORCE_THRESHOLD)
    {
        return result;
    }

    result.x = sum_Fx / sum_F;
    result.y = sum_Fy / sum_F;
    result.valid = true;

    return result;
}

// ============================================================
// 避免 Mbed printf 的 %f 問題
// ============================================================
void print_x100(int value_x100)
{
    int integer_part = value_x100 / 100;
    int decimal_part = value_x100 % 100;

    if (decimal_part < 0)
    {
        decimal_part = -decimal_part;
    }

    if (value_x100 < 0 && integer_part == 0)
    {
        printf("-");
    }

    printf("%d.%02d", integer_part, decimal_part);
}

// ============================================================
// 更新某一隻腳的最新資料
// ============================================================
void update_foot_data(
    FootData &foot,
    const uint16_t pressures[PRESSURE_COUNT],
    const COPResult &cop,
    long long now_ms
)
{
    for (int i = 0; i < PRESSURE_COUNT; i++)
    {
        foot.pressure[i] = pressures[i];
    }

    if (foot.frame_count > 0)
    {
        long long interval_ms = now_ms - foot.last_timestamp_ms;
        foot.total_interval_ms += interval_ms;

        if (interval_ms > foot.max_interval_ms)
        {
            foot.max_interval_ms = interval_ms;
        }
    }

    foot.timestamp_ms = now_ms;
    foot.last_timestamp_ms = now_ms;
    foot.frame_count++;

    foot.cop_valid = cop.valid;
    foot.cop_x = cop.x;
    foot.cop_y = cop.y;
    foot.total_force = cop.total_force;
}

// ============================================================
// 輸出一筆 CSV
// 格式：
// DATA,L/R,time_ms,P1...P18,Total_g,COP_valid,COP_X_mm,COP_Y_mm
// ============================================================
void output_frame_csv(
    uint8_t foot_id,
    long long now_ms,
    const uint16_t pressures[PRESSURE_COUNT],
    const COPResult &cop
)
{
    char foot_char = (foot_id == 0x01) ? 'L' : 'R';

    printf("DATA,%c,%lld", foot_char, now_ms);

    for (int i = 0; i < PRESSURE_COUNT; i++)
    {
        printf(",%u", pressures[i]);
    }

    printf(",%d,%d,", (int)cop.total_force, cop.valid ? 1 : 0);

    if (cop.valid)
    {
        print_x100((int)(cop.x * 100.0f));
        printf(",");
        print_x100((int)(cop.y * 100.0f));
    }
    else
    {
        // COP 無效時留空，避免把 (0,0) 誤認成真正 COP
        printf(",");
    }

    printf("\r\n");
}

// ============================================================
// 收到完整 39 bytes 後處理
// ============================================================
void process_frame()
{
    if (!check_checksum(frame))
    {
        bad_checksum++;
        printf("BAD CHECKSUM #%lu\r\n", bad_checksum);
        return;
    }

    uint8_t foot_id = frame[1];

    if (foot_id != 0x01 && foot_id != 0x02)
    {
        return;
    }

    if (!timer_started)
    {
        timer.reset();
        timer.start();
        timer_started = true;

        printf("\r\n收到第一個完整 frame，開始計時\r\n");
    }

    long long now_ms = get_elapsed_ms();

    uint16_t pressures[PRESSURE_COUNT];

    for (int i = 0; i < PRESSURE_COUNT; i++)
    {
        int index = 2 + i * 2;

        pressures[i] =
            ((uint16_t)frame[index] << 8) |
            frame[index + 1];
    }

    COPResult cop = calculate_cop(pressures, foot_id);

    if (foot_id == 0x01)
    {
        update_foot_data(left_foot, pressures, cop, now_ms);
    }
    else
    {
        update_foot_data(right_foot, pressures, cop, now_ms);
    }

    total_good_frames++;

    // 每一包都輸出，之後可直接改成寫 microSD
    output_frame_csv(foot_id, now_ms, pressures, cop);

    // 每 100 包只印一次簡短統計，避免額外 printf 太多
    if (total_good_frames % 100 == 0)
    {
        printf(
            "STATUS,Total=%lu,Left=%lu,Right=%lu,BadChecksum=%lu\r\n",
            total_good_frames,
            left_foot.frame_count,
            right_foot.frame_count,
            bad_checksum
        );
    }
}

// ============================================================
// 一個 byte 一個 byte餵進 parser
// ============================================================
void parse_byte(uint8_t b)
{
    // 等待 frame 起始 0xAA
    if (frame_index == 0)
    {
        if (b == 0xAA)
        {
            frame[0] = b;
            frame_index = 1;
        }
        return;
    }

    // 第二 byte 必須為 LEFT(01) 或 RIGHT(02)
    if (frame_index == 1)
    {
        if (b == 0x01 || b == 0x02)
        {
            frame[1] = b;
            frame_index = 2;
        }
        else if (b == 0xAA)
        {
            frame[0] = 0xAA;
            frame_index = 1;
        }
        else
        {
            frame_index = 0;
        }

        return;
    }

    frame[frame_index] = b;
    frame_index++;

    if (frame_index == FRAME_SIZE)
    {
        process_frame();
        frame_index = 0;
    }
}

// ============================================================
// main
// ============================================================
int main()
{
    esp32.set_blocking(false);

    printf("\r\n");
    printf("========================================\r\n");
    printf("Dual Foot Sensor -> ESP32 -> STM32\r\n");
    printf("39-byte Pressure Sensor Receiver\r\n");
    printf("========================================\r\n");
    printf("UART: PA10 RX, 115200 baud\r\n");
    printf("等待 LEFT / RIGHT 感測器資料...\r\n");

    uint8_t rx_buffer[64];

    while (true)
    {
        if (esp32.readable())
        {
            ssize_t count = esp32.read(rx_buffer, sizeof(rx_buffer));

            if (count > 0)
            {
                for (ssize_t i = 0; i < count; i++)
                {
                    parse_byte(rx_buffer[i]);
                }
            }
        }
    }
}
