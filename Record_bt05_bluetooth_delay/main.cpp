#include "mbed.h"
#include <stdint.h>

BufferedSerial bluetooth(PA_9, PA_10, 115200);
BufferedSerial pc(USBTX, USBRX, 115200);

// =====================================================
// Sensor protocol
// =====================================================
const int PACKET_SIZE = 39;
const int TARGET_EVENTS = 100;

const uint8_t SENSOR_HEADER_1 = 0xAA;
const uint8_t SENSOR_HEADER_2 = 0x01;

// =====================================================
// BT05 inserted text
// =====================================================
const char BT_HEADER[] = "ATT_HANDLE_VALUE_NOTI\r\n";
const int BT_HEADER_LEN = sizeof(BT_HEADER) - 1;

uint8_t bt_header_buffer[BT_HEADER_LEN];
int bt_header_index = 0;

// =====================================================
// Sensor frame
// =====================================================
uint8_t packet[PACKET_SIZE];
int packet_index = 0;

bool receiving_packet = false;
bool previous_was_AA = false;

// =====================================================
// Statistics
// =====================================================
int good_packets = 0;
int bad_packets = 0;
int incomplete_packets = 0;

int total_events = 0;
int notification_count = 0;

unsigned long long raw_bytes = 0;
unsigned long long payload_bytes = 0;

// =====================================================
// INCOMPLETE length distribution
//
// index = 收到幾 bytes 就被下一個 AA 01 打斷
// =====================================================
int incomplete_length_count[PACKET_SIZE] = {0};

// =====================================================
// HEX print
// =====================================================
void print_hex(const uint8_t *data, int len)
{
    for (int i = 0; i < len; i++)
    {
        printf("%02X ", data[i]);
    }

    printf("\r\n");
}

// =====================================================
// Checksum
// =====================================================
bool check_checksum(const uint8_t *data)
{
    uint8_t checksum = 0;

    for (int i = 0; i < 38; i++)
    {
        checksum += data[i];
    }

    return checksum == data[38];
}

// =====================================================
// Start new frame
// =====================================================
void start_new_packet()
{
    packet[0] = SENSOR_HEADER_1;
    packet[1] = SENSOR_HEADER_2;

    packet_index = 2;
    receiving_packet = true;

    previous_was_AA = false;

    printf("\r\n>>> 找到新的 AA 01，開始新封包\r\n");
}

// =====================================================
// Normal 39-byte frame completed
// =====================================================
void finish_packet()
{
    total_events++;

    printf("\r\n");
    printf("========================================\r\n");
    printf("Frame Event #%d\r\n", total_events);
    printf("========================================\r\n");

    printf("Length : 39 bytes\r\n");

    printf("HEX    : ");
    print_hex(packet, PACKET_SIZE);

    uint8_t calculated_checksum = 0;

    for (int i = 0; i < 38; i++)
    {
        calculated_checksum += packet[i];
    }

    printf(
        "Calculated Checksum : %02X\r\n",
        calculated_checksum
    );

    printf(
        "Received Checksum   : %02X\r\n",
        packet[38]
    );

    if (check_checksum(packet))
    {
        good_packets++;
        printf("Result : GOOD\r\n");
    }
    else
    {
        bad_packets++;
        printf("Result : CHECKSUM ERROR\r\n");
    }

    printf(
        "GOOD=%d | BAD=%d | INCOMPLETE=%d\r\n",
        good_packets,
        bad_packets,
        incomplete_packets
    );

    receiving_packet = false;
    packet_index = 0;
    previous_was_AA = false;
}

// =====================================================
// Frame interrupted by another AA 01
// =====================================================
void incomplete_packet_detected(int old_length)
{
    incomplete_packets++;
    total_events++;

    // ---------------------------------------------
    // 統計這個 incomplete 的長度
    // ---------------------------------------------
    if (old_length >= 0 &&
        old_length < PACKET_SIZE)
    {
        incomplete_length_count[old_length]++;
    }

    printf("\r\n");
    printf("========================================\r\n");
    printf("Frame Event #%d\r\n", total_events);
    printf("========================================\r\n");

    printf("Result : INCOMPLETE\r\n");

    printf(
        "Received Length : %d / 39 bytes\r\n",
        old_length
    );

    printf("Partial HEX     : ");
    print_hex(packet, old_length);

    printf(
        ">>> 在封包尚未完成時又出現新的 AA 01\r\n"
    );

    printf(
        "GOOD=%d | BAD=%d | INCOMPLETE=%d\r\n",
        good_packets,
        bad_packets,
        incomplete_packets
    );
}

// =====================================================
// Sensor stream parser
// =====================================================
void process_sensor_byte(uint8_t c)
{
    payload_bytes++;

    // =================================================
    // 尚未找到 frame
    // =================================================
    if (!receiving_packet)
    {
        if (previous_was_AA)
        {
            // 找到 AA 01
            if (c == SENSOR_HEADER_2)
            {
                start_new_packet();
                return;
            }

            // AA AA
            if (c == SENSOR_HEADER_1)
            {
                previous_was_AA = true;
                return;
            }

            previous_was_AA = false;
            return;
        }

        if (c == SENSOR_HEADER_1)
        {
            previous_was_AA = true;
        }

        return;
    }

    // =================================================
    // 已經在收 frame
    // 同時監視新的 AA 01
    // =================================================
    if (previous_was_AA)
    {
        if (c == SENSOR_HEADER_2)
        {
            // 上一個 AA 已經寫進 packet
            // 但它其實是新 frame 的開頭
            // 所以前一包長度要 -1
            int old_length = packet_index - 1;

            if (old_length < 0)
            {
                old_length = 0;
            }

            incomplete_packet_detected(old_length);

            // 從新的 AA 01 立刻重新同步
            start_new_packet();

            return;
        }

        previous_was_AA = false;
    }

    // =================================================
    // 正常加入目前 frame
    // =================================================
    if (packet_index < PACKET_SIZE)
    {
        packet[packet_index++] = c;
    }

    // 記錄這個 byte 是否為 AA
    if (c == SENSOR_HEADER_1)
    {
        previous_was_AA = true;
    }

    // =================================================
    // 收滿 39 bytes
    // =================================================
    if (packet_index >= PACKET_SIZE)
    {
        finish_packet();
    }
}

// =====================================================
// Remove ATT_HANDLE_VALUE_NOTI\r\n
// =====================================================
void process_bt_byte(uint8_t c)
{
    if (bt_header_index == 0)
    {
        if (c == (uint8_t)BT_HEADER[0])
        {
            bt_header_buffer[0] = c;
            bt_header_index = 1;
        }
        else
        {
            process_sensor_byte(c);
        }

        return;
    }

    // 正在比對 BT header
    if (c == (uint8_t)BT_HEADER[bt_header_index])
    {
        bt_header_buffer[bt_header_index] = c;

        bt_header_index++;

        if (bt_header_index == BT_HEADER_LEN)
        {
            notification_count++;
            bt_header_index = 0;
        }

        return;
    }

    // =================================================
    // 比對失敗
    // 暫存的 byte 其實是 payload
    // =================================================
    for (int i = 0; i < bt_header_index; i++)
    {
        process_sensor_byte(
            bt_header_buffer[i]
        );
    }

    bt_header_index = 0;

    if (c == (uint8_t)BT_HEADER[0])
    {
        bt_header_buffer[0] = c;
        bt_header_index = 1;
    }
    else
    {
        process_sensor_byte(c);
    }
}

// =====================================================
// Final result
// =====================================================
void print_result()
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("Dynamic Resync Final Result\r\n");
    printf("========================================\r\n");

    printf(
        "Total Frame Events : %d\r\n",
        total_events
    );

    printf(
        "GOOD               : %d\r\n",
        good_packets
    );

    printf(
        "Checksum ERROR     : %d\r\n",
        bad_packets
    );

    printf(
        "INCOMPLETE         : %d\r\n",
        incomplete_packets
    );

    printf(
        "Notifications      : %d\r\n",
        notification_count
    );

    printf(
        "UART Raw Bytes     : %llu\r\n",
        raw_bytes
    );

    printf(
        "Sensor Payload     : %llu\r\n",
        payload_bytes
    );

    // =================================================
    // INCOMPLETE Length distribution
    // =================================================
    printf("\r\n");
    printf("========================================\r\n");
    printf("INCOMPLETE Length Distribution\r\n");
    printf("========================================\r\n");

    for (int i = 0; i < PACKET_SIZE; i++)
    {
        if (incomplete_length_count[i] > 0)
        {
            printf(
                "%2d bytes : %d times\r\n",
                i,
                incomplete_length_count[i]
            );
        }
    }

    printf("========================================\r\n");
}

// =====================================================
// main
// =====================================================
int main()
{
    bluetooth.set_blocking(false);

    printf("\r\n");
    printf("========================================\r\n");
    printf("BT05 Dynamic Frame + Length Analyzer\r\n");
    printf("========================================\r\n");

    printf("UART         : 115200 8N1\r\n");
    printf("Frame Header : AA 01\r\n");
    printf("Frame Size   : 39 Bytes\r\n");

    printf(
        "Target Events: %d\r\n",
        TARGET_EVENTS
    );

    printf("\r\n等待資料...\r\n");

    while (true)
    {
        if (bluetooth.readable())
        {
            uint8_t c;

            if (bluetooth.read(&c, 1) > 0)
            {
                raw_bytes++;

                process_bt_byte(c);

                if (total_events >= TARGET_EVENTS)
                {
                    print_result();

                    while (true)
                    {
                        ThisThread::sleep_for(1s);
                    }
                }
            }
        }
    }
}