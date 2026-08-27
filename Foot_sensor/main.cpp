#include "mbed.h"
#include <stdlib.h>
#include <string.h>
#include <chrono>

BufferedSerial esp32(PA_9, PA_10, 115200);
BufferedSerial pc(USBTX, USBRX, 115200);
Timer timer;

bool testing = false;
int target_hz = 0;

int expected_num = 1;
int received_count = 0;
int lost_count = 0;
int duplicate_count = 0;

int first_sequence = 0;
int last_sequence = 0;

long long last_packet_time_us = 0;
long long total_interval_us = 0;
long long max_interval_us = 0;
long long max_lag_us = 0;

long long get_elapsed_us()
{
    return std::chrono::duration_cast<
        std::chrono::microseconds
    >(timer.elapsed_time()).count();
}

void print_x100(long long value_x100)
{
    long long integer_part = value_x100 / 100;
    long long decimal_part = value_x100 % 100;

    if (decimal_part < 0)
        decimal_part = -decimal_part;

    printf("%lld.%02lld", integer_part, decimal_part);
}

void print_us_as_ms_x100(long long value_us)
{
    long long ms_x100 = value_us / 10;
    print_x100(ms_x100);
}

void reset_test()
{
    testing = false;

    expected_num = 1;
    received_count = 0;
    lost_count = 0;
    duplicate_count = 0;

    first_sequence = 0;
    last_sequence = 0;

    last_packet_time_us = 0;
    total_interval_us = 0;
    max_interval_us = 0;
    max_lag_us = 0;

    timer.stop();
    timer.reset();
}

void print_result(int sent_total)
{
    timer.stop();

    long long elapsed_us = get_elapsed_us();

    if (sent_total > 0 && last_sequence < sent_total)
    {
        lost_count += sent_total - last_sequence;
    }

    long long loss_rate_x100 = 0;

    if (sent_total > 0)
    {
        loss_rate_x100 =
            ((long long)lost_count * 10000LL) / sent_total;
    }

    long long receive_hz_x100 = 0;

    if (elapsed_us > 0 && received_count > 1)
    {
        receive_hz_x100 =
            ((long long)(received_count - 1) * 100000000LL)
            / elapsed_us;
    }

    long long avg_interval_us = 0;

    if (received_count > 1)
    {
        avg_interval_us =
            total_interval_us / (received_count - 1);
    }

    printf("\r\n");
    printf("========================================\r\n");
    printf("PC BLE -> ESP32 -> STM32 測試結果\r\n");
    printf("========================================\r\n");
    printf("目標頻率       : %d Hz\r\n", target_hz);
    printf("理論總包數     : %d\r\n", sent_total);
    printf("實際收到       : %d\r\n", received_count);
    printf("掉包數         : %d\r\n", lost_count);
    printf("重複/亂序包    : %d\r\n", duplicate_count);

    printf("掉包率         : ");
    print_x100(loss_rate_x100);
    printf(" %%\r\n");

    printf("實際接收頻率   : ");
    print_x100(receive_hz_x100);
    printf(" Hz\r\n");

    printf(
        "總接收時間     : %lld.%06lld s\r\n",
        elapsed_us / 1000000,
        elapsed_us % 1000000
    );

    printf("平均封包間隔   : ");
    print_us_as_ms_x100(avg_interval_us);
    printf(" ms\r\n");

    printf("最大封包間隔   : ");
    print_us_as_ms_x100(max_interval_us);
    printf(" ms\r\n");

    printf("最大累積落後   : ");
    print_us_as_ms_x100(max_lag_us);
    printf(" ms\r\n");

    printf("========================================\r\n");
    printf("\r\n等待下一輪 START...\r\n");

    reset_test();
}

void process_line(char *line)
{
    if (strncmp(line, "START,", 6) == 0)
    {
        int hz = atoi(line + 6);

        if (hz <= 0)
        {
            printf("START 頻率無效\r\n");
            return;
        }

        reset_test();
        target_hz = hz;

        printf("\r\n");
        printf("========================================\r\n");
        printf("收到 START\r\n");
        printf("目標頻率 : %d Hz\r\n", target_hz);
        printf("========================================\r\n");

        return;
    }

    if (strncmp(line, "END,", 4) == 0)
    {
        int sent_total = atoi(line + 4);

        if (testing)
        {
            print_result(sent_total);
        }
        else
        {
            printf("收到 END，但尚未收到測試封包\r\n");
        }

        return;
    }

    size_t len = strlen(line);

    if (len != 38)
    {
        return;
    }

    int current_num = atoi(line);

    if (current_num <= 0)
    {
        return;
    }

    if (!testing)
    {
        timer.reset();
        timer.start();

        testing = true;

        first_sequence = current_num;
        expected_num = current_num;
        last_packet_time_us = 0;

        printf(
            "收到第一包 #%d，開始計時\r\n",
            current_num
        );
    }

    long long now_us = get_elapsed_us();

    if (current_num > expected_num)
    {
        int missing = current_num - expected_num;
        lost_count += missing;
    }
    else if (current_num < expected_num)
    {
        duplicate_count++;
        return;
    }

    if (received_count > 0)
    {
        long long interval_us =
            now_us - last_packet_time_us;

        total_interval_us += interval_us;

        if (interval_us > max_interval_us)
        {
            max_interval_us = interval_us;
        }
    }

    last_packet_time_us = now_us;

    received_count++;
    last_sequence = current_num;
    expected_num = current_num + 1;

    if (target_hz > 0)
    {
        long long theoretical_us =
            ((long long)(current_num - first_sequence)
             * 1000000LL)
            / target_hz;

        long long lag_us =
            now_us - theoretical_us;

        if (lag_us > max_lag_us)
        {
            max_lag_us = lag_us;
        }
    }

    if (received_count % 100 == 0)
    {
        printf(
            "[RX %d] Seq=%d Loss=%d\r\n",
            received_count,
            current_num,
            lost_count
        );
    }
}

int main()
{
    esp32.set_blocking(false);

    printf("\r\n");
    printf("========================================\r\n");
    printf("PC BLE -> ESP32 -> STM32\r\n");
    printf("39-byte Bandwidth Receiver\r\n");
    printf("========================================\r\n");
    printf("UART: PA10 RX, 115200 baud\r\n");
    printf("等待 START...\r\n");

    char rx_buf[64];
    size_t rx_idx = 0;

    while (true)
    {
        if (esp32.readable())
        {
            char c;

            if (esp32.read(&c, 1) > 0)
            {
                if (c == '\n')
                {
                    rx_buf[rx_idx] = '\0';
                    process_line(rx_buf);
                    rx_idx = 0;
                }
                else if (c != '\r')
                {
                    if (rx_idx < sizeof(rx_buf) - 1)
                    {
                        rx_buf[rx_idx++] = c;
                    }
                    else
                    {
                        rx_idx = 0;
                    }
                }
            }
        }
    }
}
