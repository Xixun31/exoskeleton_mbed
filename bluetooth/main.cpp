#include "mbed.h"

// --- 藍牙 UART 設定 (Mbed OS 6+ 寫法) ---
// PA_9 (TX), PA_10 (RX), Baud Rate 設為 115200
static UnbufferedSerial bt_serial(PA_9, PA_10, 9600);

// --- 接收資料用的變數 ---
char myRxData[6] = {0};
int rx_index = 0;
bool ITflag = false;

// 函式宣告
void Rx_interrupt();
void send_bt_data();

int main() {
    // 預設將標準 printf 輸出到電腦端 USB (方便你看 Debug 訊息)
    printf("Bluetooth Only Program Started!\r\n");

    // 設定藍牙接收中斷：只要 PA_10 收到訊號，就會立刻跳去執行 Rx_interrupt
    bt_serial.attach(&Rx_interrupt, SerialBase::RxIrq);

    while (true) {
        thread_sleep_for(500); // 為了不讓傳輸太頻繁，設定每 500 毫秒執行一次
        
        // 1. 測試：主動傳送資料給藍牙模組
        send_bt_data();

        // 2. 測試：如果在中斷裡收滿了 6 個 Byte，就在電腦端印出來確認
        if (ITflag) {
            printf("收到藍牙資料: %d, %d, %d, %d, %d, %d\r\n", 
                   myRxData[0], myRxData[1], myRxData[2], 
                   myRxData[3], myRxData[4], myRxData[5]);
                   
            ITflag = false; // 處理完畢，把旗標降下，等待下一包 6 Bytes
        }
    }
}

// --- 藍牙接收中斷函式 ---
// 注意：中斷裡面越短越好，絕對不能放 wait() 或 printf()
void Rx_interrupt() {
    char c;
    
    // 讀取單一個 Byte (Mbed OS 6 安全寫法)
    if (bt_serial.read(&c, 1)) {
        myRxData[rx_index] = c; // 把收到的字元存進陣列
        rx_index++;             // 索引值 +1
        
        // 如果已經收滿 6 個 Byte
        if (rx_index >= 6) {
            rx_index = 0;   // 歸零，準備下一次接收
            ITflag = true;  // 立起旗標，通知 main() 可以處理這包資料了
        }
    }
}

// --- 藍牙發送函式 ---
void send_bt_data() {
    // 建立要傳送的字串
    char tx_msg[] = "Hello from Mbed BT!\r\n";
    
    // 透過藍牙 TX (PA_9) 發送出去
    // sizeof(tx_msg) - 1 是為了不把字串結尾的 '\0' (Null terminator) 傳出去
    bt_serial.write(tx_msg, sizeof(tx_msg) - 1);
}