#include "mbed.h"
#include <chrono>

// ============================================================
// UART
//
// ESP32 GPIO17 TX -> STM32 PA10 RX
// ESP32 GND       -> STM32 GND
//
// PA9 是 STM32 TX，這次其實不用接
// ============================================================

BufferedSerial esp32(PA_9, PA_10, 115200);

BufferedSerial pc(USBTX, USBRX, 115200);


// ============================================================
// 感測器 frame
// ============================================================

const int FRAME_SIZE = 39;
const int PRESSURE_COUNT = 18;

uint8_t frame[FRAME_SIZE];

int frame_index = 0;


// ============================================================
// 測試統計
// ============================================================

const int TOTAL_PACKETS = 500;

Timer timer;

bool testing = false;

int good_frames = 0;
int bad_checksum = 0;

long long max_interval_ms = 0;
long long total_interval_ms = 0;

long long last_frame_time_ms = 0;


// ============================================================
// Timer
// ============================================================

long long get_elapsed_ms()
{
    return std::chrono::duration_cast<
        std::chrono::milliseconds
    >(
        timer.elapsed_time()
    ).count();
}


// ============================================================
// Checksum
//
// byte[38] =
// byte[0] ~ byte[37] 加總後低8位
// ============================================================

bool check_checksum(uint8_t *data)
{
    uint16_t sum = 0;

    for (int i = 0; i < 38; i++)
    {
        sum += data[i];
    }

    uint8_t checksum =
        (uint8_t)(sum & 0xFF);

    return checksum == data[38];
}


// ============================================================
// 印出一個完整 frame
// ============================================================

void print_frame(uint8_t *data)
{
    // --------------------------------------------------------
    // 左右腳
    // --------------------------------------------------------

    printf("Foot=");

    if (data[1] == 0x01)
    {
        printf("LEFT");
    }
    else if (data[1] == 0x02)
    {
        printf("RIGHT");
    }
    else
    {
        printf("UNKNOWN");
    }


    printf(" | ");


    // --------------------------------------------------------
    // 18個 pressure
    //
    // 每點2 bytes：
    // High byte + Low byte
    // --------------------------------------------------------

    for (int i = 0; i < PRESSURE_COUNT; i++)
    {
        int index =
            2 + i * 2;


        uint16_t pressure =
            ((uint16_t)data[index] << 8)
            |
            data[index + 1];


        printf(
            "P%d=%u",
            i + 1,
            pressure
        );


        if (i != PRESSURE_COUNT - 1)
        {
            printf(", ");
        }
    }

    printf("\r\n");
}


// ============================================================
// 印最後統計
// ============================================================

void print_result()
{
    timer.stop();

    long long elapsed_ms =
        get_elapsed_ms();


    int hz_x100 = 0;

    if (
        elapsed_ms > 0 &&
        good_frames > 1
    )
    {
        hz_x100 =
            (int)(
                ((long long)
                (good_frames - 1)
                * 100000LL)
                /
                elapsed_ms
            );
    }


    long long avg_interval_ms = 0;

    if (good_frames > 1)
    {
        avg_interval_ms =
            total_interval_ms
            /
            (good_frames - 1);
    }


    printf("\r\n");
    printf("========================================\r\n");
    printf("Sensor -> ESP32 -> STM32 測試結果\r\n");
    printf("========================================\r\n");

    printf(
        "有效封包       : %d\r\n",
        good_frames
    );

    printf(
        "Checksum Error : %d\r\n",
        bad_checksum
    );

    printf(
        "總接收時間     : %lld.%03lld s\r\n",
        elapsed_ms / 1000,
        elapsed_ms % 1000
    );


    printf(
        "實際接收頻率   : %d.%02d Hz\r\n",
        hz_x100 / 100,
        hz_x100 % 100
    );


    printf(
        "平均封包間隔   : %lld ms\r\n",
        avg_interval_ms
    );


    printf(
        "最大封包間隔   : %lld ms\r\n",
        max_interval_ms
    );


    printf("========================================\r\n");
}


// ============================================================
// 收到完整39 bytes
// ============================================================

void process_frame()
{
    // ========================================================
    // checksum
    // ========================================================

    if (!check_checksum(frame))
    {
        bad_checksum++;

        printf(
            "BAD CHECKSUM #%d\r\n",
            bad_checksum
        );

        return;
    }


    // ========================================================
    // 第一個有效 frame
    // ========================================================

    if (!testing)
    {
        timer.reset();
        timer.start();

        testing = true;

        last_frame_time_ms = 0;

        printf("\r\n");
        printf("收到第一個完整 frame，開始計時\r\n");
    }


    // ========================================================
    // 時間統計
    // ========================================================

    long long now_ms =
        get_elapsed_ms();


    if (good_frames > 0)
    {
        long long interval_ms =
            now_ms -
            last_frame_time_ms;


        total_interval_ms +=
            interval_ms;


        if (
            interval_ms >
            max_interval_ms
        )
        {
            max_interval_ms =
                interval_ms;
        }
    }


    last_frame_time_ms =
        now_ms;


    good_frames++;


    // ========================================================
    // 印 pressure
    // ========================================================

    printf(
        "[Frame %d] ",
        good_frames
    );

    print_frame(frame);


    // ========================================================
    // 500包完成
    // ========================================================

    if (
        good_frames >=
        TOTAL_PACKETS
    )
    {
        print_result();


        // 停在這裡
        while (true)
        {
            ThisThread::sleep_for(
                1s
            );
        }
    }
}


// ============================================================
// 一個 byte 一個 byte餵進 parser
// ============================================================

void parse_byte(uint8_t b)
{
    // ========================================================
    // 等待 AA
    // ========================================================

    if (frame_index == 0)
    {
        if (b == 0xAA)
        {
            frame[0] = b;

            frame_index = 1;
        }

        return;
    }


    // ========================================================
    // 第二 byte 必須 01 或 02
    // ========================================================

    if (frame_index == 1)
    {
        if (
            b == 0x01 ||
            b == 0x02
        )
        {
            frame[1] = b;

            frame_index = 2;
        }
        else if (b == 0xAA)
        {
            // 又遇到新的 AA
            // 保留作為下一個 frame 開頭

            frame[0] = 0xAA;

            frame_index = 1;
        }
        else
        {
            frame_index = 0;
        }

        return;
    }


    // ========================================================
    // 收剩下的 bytes
    // ========================================================

    frame[frame_index] = b;

    frame_index++;


    // ========================================================
    // 收滿39 bytes
    // ========================================================

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
    printf("Sensor -> ESP32 -> STM32\r\n");
    printf("39-byte Pressure Sensor Receiver\r\n");
    printf("========================================\r\n");

    printf(
        "UART: PA10 RX, 115200 baud\r\n"
    );

    printf(
        "等待 ESP32 傳送感測器資料...\r\n"
    );


    // ========================================================
    // 暫存 UART 一次讀到的資料
    //
    // 注意：
    // ESP32 雖然 write(frame, 39)
    // STM32 read() 不保證一次就是39 bytes
    // 所以一次最多讀64，再逐byte解析
    // ========================================================

    uint8_t rx_buffer[64];


    while (true)
    {
        if (esp32.readable())
        {
            ssize_t count =
                esp32.read(
                    rx_buffer,
                    sizeof(rx_buffer)
                );


            if (count > 0)
            {
                for (
                    ssize_t i = 0;
                    i < count;
                    i++
                )
                {
                    parse_byte(
                        rx_buffer[i]
                    );
                }
            }
        }
    }
}