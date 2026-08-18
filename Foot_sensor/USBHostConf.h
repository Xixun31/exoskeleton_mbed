#ifndef USBHOST_CONF_H
#define USBHOST_CONF_H

/*
* 允許連接到 USB Host 的最大設備數量
*/
#define MAX_DEVICE_CONNECTED        5

/*
* 允許連接的最大 USB Hub 數量
*/
#define MAX_HUB_NB                  2

/*
* USB Hub 上的最大連接埠數量
*/
#define MAX_HUB_PORT                4

/*
* 啟用隨身碟 (Mass Storage Device) 支援
*/
#define USBHOST_MSD                 0

/*
* 啟用鍵盤支援
*/
#define USBHOST_KEYBOARD            0

/*
* 啟用滑鼠支援
*/
#define USBHOST_MOUSE               0

/*
* 啟用虛擬串口 (Serial) 支援 -> 這是你目前讀取藍牙接收器需要的！
*/
#define USBHOST_SERIAL              1

/*
* 啟用 3G 模組支援
*/
#define USBHOST_3GMODULE            0

/*
* 啟用 MIDI 支援
*/
#define USBHOST_MIDI                0

#endif