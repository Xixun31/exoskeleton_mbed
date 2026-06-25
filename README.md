# Exoskeleton Mbed Project

這個項目使用 Mbed OS 開發外骨骼系統，包含多個模塊如藍牙、編碼器、IMU 等。

## 要求

- Mbed CLI
- GCC ARM 工具鏈 (推薦 GCC 9)
- NUCLEO-F446RE 開發板
- USB 驅動與序列埠通訊工具 (`screen` 或其他序列埠終端)

---

## 安裝與設置

1. **確保 `mbed-os` 目錄存在**（位於 `/home/xixun/project/mbed/mbed-os`）。
2. **在專案目錄中創建 `mbed-os` 的符號連結**：
   ```bash
   ln -s ../mbed-os .
   ```
3. **複製必要設定檔與描述檔**：
   ```bash
   cp ../imu_uart/mbed_app.json .
   cp ../imu_uart/.mbed .
   ```
4. **疑難排解 (解決 Library 衝突與編譯錯誤)**：
   為了避免 Mbed CLI 掃描到 `mbed-os` 內部的測試與模擬器目錄而發生編譯衝突，必須在 `mbed-os` 根目錄下放置 `.mbedignore` 檔案。專案內已設定好全域的 [mbed-os/.mbedignore](file:///home/xixun/project/mbed/mbed-os/.mbedignore)。

---

## 編譯

切換至專案資料夾後，執行以下命令編譯：

### macOS
```bash
mbed compile -m NUCLEO_F446RE -t GCC_ARM -f
```

### Ubuntu / Linux
```bash
mbed compile -m NUCLEO_F446RE -t GCC_ARM
```
這會生成二進位檔案於 `BUILD/NUCLEO_F446RE/GCC_ARM/` 目錄中。若要進行乾淨編譯（清除快取），可加入 `-c` 參數。

---

## 燒錄

將生成的 `.bin` 檔案複製到 NUCLEO 開發板的虛擬磁碟：

### macOS
```bash
cp -X BUILD/NUCLEO_F446RE/GCC_ARM/*.bin /Volumes/NOD_F446RE/
```

### Ubuntu / Linux
```bash
cp ./BUILD/NUCLEO_F446RE/GCC_ARM/*.bin /media/xixun/NOD_F446RE/ && sync
```

---

## 串口通信

開發板的 `mbed_app.json` 已將 stdio 波特率設定為 `115200`。

### macOS
1. **尋找設備：**
   ```bash
   ls /dev/cu.usb*
   ```
2. **連線：**
   ```bash
   screen /dev/cu.usbmodem103 115200
   ```

### Ubuntu / Linux
1. **尋找設備：**
   ```bash
   ls -la /dev/ttyACM*
   ```
   *（一般連接後為 `/dev/ttyACM0`）*
2. **使用者群組與權限設定：**
   若遇到 `Permission denied`，請將當前使用者加入 `dialout` 群組（設定後需重新登入系統以生效）：
   ```bash
   sudo usermod -aG dialout $USER
   ```
   或者暫時設定設備權限：
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```
3. **連線：**
   ```bash
   screen /dev/ttyACM0 115200
   ```

---

## PlatformIO 編譯燒錄說明

由於本工作區包含多個實驗專案，若要使用 PlatformIO 進行特定的專案編譯與燒錄，請務必先修改最外層的 [platformio.ini](file:///home/xixun/project/mbed/platformio.ini) 設定檔。

請將設定檔中的 `src_dir` 修改為 **您想要編譯與燒錄的資料夾名稱**（例如：`src_dir = Test_NUCLEO_F446RE`）。修改完成並存檔後，再按下 PlatformIO 的編譯或上傳（Upload）按鈕。

---

## 模塊說明

- `bluetooth/`: 藍牙通信模組
- `encoder/`: 編碼器模組
- `IMU/`: 慣性測量單元模組
- `imu_uart/`: UART 通信的 IMU 模組
- `mbed-os5-passive/`: 被動模式下的 Mbed OS 5
- `Test_NUCLEO_F446RE/`: 測試模組
