#include "mbed.h"
#include <string.h>

// 設定藍牙與電腦的序列埠
BufferedSerial bluetooth(PA_9, PA_10, 9600); 
BufferedSerial pc(USBTX, USBRX, 115200);

int main() {
    printf("--- HM-10 Auto Setup & Smart Passthrough ---\n");
    printf("Step 1: Auto-configuring as Master...\n");

    // 1. 自動發送 AT 測試通訊
    const char cmd1[] = "AT\r\n";
    bluetooth.write(cmd1, strlen(cmd1));
    thread_sleep_for(500);

    // 2. 自動切換為主機模式
    const char cmd2[] = "AT+ROLE1\r\n"; 
    bluetooth.write(cmd2, strlen(cmd2));
    thread_sleep_for(500);

    // 3. 自動重啟讓設定生效
    const char cmd3[] = "AT+RESET\r\n";
    bluetooth.write(cmd3, strlen(cmd3));
    thread_sleep_for(800); 

    printf("Step 2: Setup Done! Now entering SMART AT command mode.\n");
    printf("Just type your command in VS Code and press ENTER.\n");
    printf("--------------------------------------------------\n");

    uint8_t bt_buf[1];
    char cmd_buffer[64]; // 用來收集你打的字的「緩衝籃子」
    int cmd_index = 0;

    while (1) {
        // 1. 處理電腦 (VS Code) 輸入
        if (pc.readable()) {
            uint8_t c;
            pc.read(&c, 1);
            pc.write(&c, 1); // 讓你在終端機看得到自己打了什麼字 (Echo回顯)

            if (c == '\r' || c == '\n') {
                // 當你按下 Enter，把收集到的字串加上 \r\n，整包送給藍牙
                if (cmd_index > 0) {
                    cmd_buffer[cmd_index++] = '\r';
                    cmd_buffer[cmd_index++] = '\n';
                    bluetooth.write(cmd_buffer, cmd_index);
                    cmd_index = 0; // 清空籃子，準備裝下一次的指令
                }
            } else {
                // 還沒按 Enter 前，把字元一個個裝進籃子裡
                if (cmd_index < 60) {
                    cmd_buffer[cmd_index++] = c;
                }
            }
        }
        
        // 2. 處理藍牙模組回傳 (把它印在 VS Code 終端機上)
        if (bluetooth.readable()) {
            bluetooth.read(bt_buf, 1);
            pc.write(bt_buf, 1);
        }
    }
}
//left: AT+BANDFF2405103703
//right: AT+BANDFF240510831F