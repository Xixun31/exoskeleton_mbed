# Exoskeleton Mbed Project

這個項目使用Mbed OS開發外骨骼系統，包含多個模塊如藍牙、編碼器、IMU等。

## 要求

- Mbed CLI
- GCC ARM工具鏈
- NUCLEO-F446RE開發板
- USB驅動（用於燒錄和串口通信）

## 安裝與設置

1. 確保mbed-os目錄存在（如果沒有，從mbed-os倉庫克隆）。

2. 在項目目錄中創建mbed-os的符號鏈接：
   ```
   ln -s ../mbed-os .
   ```

3. 複製配置文件：
   ```
   cp ../imu_uart/mbed_app.json .
   cp ../imu_uart/.mbed .
   ```

## 編譯

使用以下命令編譯項目：
```
mbed compile -m NUCLEO_F446RE -t GCC_ARM -f
```

這會生成二進制文件在BUILD/NUCLEO_F446RE/GCC_ARM/目錄中。

## 燒錄

將生成的.bin文件複製到NUCLEO板子的虛擬驅動器：
```
cp -X BUILD/NUCLEO_F446RE/GCC_ARM/*.bin /Volumes/NOD_F446RE/
```

## 串口通信

1. 檢查串口設備：
   ```
   ls /dev/cu.usb*
   ```

2. 使用screen連接：
   ```
   screen /dev/cu.usbmodem103 115200
   ```
## PlatformIO編譯燒錄說明

由於本工作區包含多個實驗專案，每次要執行或編譯特定程式碼時，請務必先修改最外層的 `platformio.ini` 設定檔。
  
請將設定檔中的 `src_dir` 修改為**你今天想要編譯與燒錄的資料夾名稱**
（例如：`src_dir = Test_NUCLEO_F446RE`）。修改完成並存檔後，再按下編譯或上傳（Upload）按鈕。

## Mbed OS知識

- Mbed OS是Arm開發的物聯網操作系統，支持多種微控制器。
- NUCLEO-F446RE是STM32F446RE的開發板，具有高性能和豐富的外設。
- 項目結構：每個子目錄代表一個模塊，包含main.cpp和mbed_app.json。
- 編譯輸出存放在BUILD/目錄中，應在.gitignore中忽略。

## 模塊說明

- bluetooth/: 藍牙通信模塊
- encoder/: 編碼器模塊
- IMU/: 慣性測量單元模塊
- imu_uart/: UART通信的IMU模塊
- mbed-os5-passive/: 被動模式下的Mbed OS 5
- Test_NUCLEO_F446RE/: 測試模塊

更多詳情請參考各模塊的main.cpp和mbed_app.json。
