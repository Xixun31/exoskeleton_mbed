# enc_imu_sd_logger 操作手冊

2 顆 AS5145 編碼器 (串接) + 1 顆 Xsens MTi IMU，以 100 Hz 同步讀取；再加上左右腳足底感測器 (BLE → ESP32 → UART)。
全部用同一個 MCU µs 時鐘打時間戳 (time code)，存到 microSD 卡，可以檢查各裝置時間有沒有對齊。
不接電腦、只接電源就能獨立紀錄；用板子上的藍色 B1 按鈕操作，綠燈 LD2 看狀態。

---

## 1. 接線

全部接在 NUCLEO-F446RE **正面的 Morpho 公頭** (CN7 = 左側、CN10 = 右側)。

### Encoder ×2 (SPI1)
| 訊號 | 腳位 | 排針 | 接到 |
|---|---|---|---|
| SCK | PB_3 | CN10-31 | encoder1 板 P2-3 (SCK) |
| MISO | PB_4 | CN10-27 | encoder1 板 P2-2 (MISO) |
| CS | PC_7 | CN10-19 | encoder1 板 P2-4 (CS) |
| 3.3V | | CN7-16 / CN7-5 | encoder1 板 P2-5 |
| GND | | CN10-20 | encoder1 板 P2-1 |

- encoder1 板的 **P1** 接 encoder2 板的 **P2**，要**反接** (P1 第 n 腳 ↔ P2 第 6−n 腳)：3.3V↔3.3V、CS↔CS、SCK↔SCK、PROG↔MISO、GND↔GND。
- **encoder2 板的 P1-4 (PROG) 接 GND** (串接最後一顆的 PDIO 要接地)。
- 編碼器板是 **3.3V 設計** (VDD5V、VDD3V3 已在板上接在一起)，**不可接 5V**。
- 資料順序：MISO 直接接到的那顆是 encoder1。

### IMU (SPI2)
| 訊號 | 腳位 | 排針 |
|---|---|---|
| SCK | PB_13 | CN10-30 |
| MISO | PB_14 | CN10-28 |
| MOSI | PB_15 | CN10-26 |
| nCS | PB_1 | CN10-24 |
| DRDY (選配) | PC_8 | CN10-2 |
| VDD 3.3V / GND | | CN7-16 或 CN7-5 / CN10-9 |

- SPI 同名腳對接 (IMU MISO → MCU MISO)。IMU 的 MISO 是 CN10-28，**旁邊的 CN10-27 是 encoder 的，不要插錯**。
- 目前的 IMU 板沒有 DRDY，不接也可以：程式會自動改成每 1 ms 輪詢 IMU，有新封包才讀 (CSV `trig` = 2)。

### microSD (SPI3)
| 訊號 | 腳位 | 排針 |
|---|---|---|
| SCK | PC_10 | CN7-1 |
| MISO | PC_11 | CN7-2 |
| MOSI | PC_12 | CN7-3 |
| CS | PD_2 | CN7-4 |
| VCC | | 有穩壓器的模組接 CN7-18 (5V)；純 3.3V 模組接 CN7-5 |
| GND | | CN7-8 / 19 / 20 |

- SD 卡格式化為 **FAT32**。
- 3.3V 腳位不夠時：CN7-5 (VDD) 跟 CN7-16 (3V3) 是同一路 3.3V。**不要用 CN7-6 (E5V)**，那是電源輸入腳。

### 足底感測器 (ESP32 → USART1)
| 訊號 | 腳位 | 排針 | 接到 |
|---|---|---|---|
| RX | PA_10 | CN10-33 | ESP32 **GPIO17** (TX) |
| TX | PA_9 | CN10-21 | 不接 (保留) |
| GND | | CN10-9 / CN7-19 / CN7-20 | ESP32 **GND** (一定要共地) |

- 兩隻腳的感測器用 BLE 連到 ESP32，ESP32 把封包從 GPIO17 轉送給 STM32 (115200 baud)。
- ESP32 要燒 `ESP32_sensor_test/` 的程式 (`SEND_TIMESTAMP = true`，封包會帶 ESP32 時間戳與編號)。
  沒重燒的舊版 ESP32 程式也能用，只是足底檔的 `esp_us` / `esp_seq` 會是空白。
- ESP32 用自己的 USB 線供電 (例如同一顆行動電源的第二個 USB 孔)；**不要從 NUCLEO 的 3.3V 供電** (BLE 電流尖峰會讓 3.3V 掉壓)。
- 舊的 `Foot_sensor` 程式把 SD 卡接在 PA5~PA7、PB6 (Arduino SPI)；這個程式的 SD 卡改在 SPI3 (CN7-1~4)，**不要照舊接法接 SD**。

### 腳位總表
| 裝置 | 介面 | 使用腳位 |
|---|---|---|
| Encoder ×2 | SPI1 | PB_3、PB_4、PC_7 |
| IMU | SPI2 | PB_13、PB_14、PB_15、PB_1、(PC_8) |
| microSD | SPI3 | PC_10、PC_11、PC_12、PD_2 |
| 足底 ESP32 | USART1 | PA_10 (PA_9 保留) |
| 板上 | | PA_2/PA_3 序列埠 (USB)、PA_5 LD2、PC_13 B1、PA_13/PA_14 SWD |

---

## 2. 電源 (不接電腦獨立運作)

整套 (板子 + 2 encoder + IMU + SD) 約 150~300 mA；ESP32 另外約 100~250 mA (建議 ESP32 自己用一條 USB 供電)。擇一：

| 方式 | 接法 | 跳線 |
|---|---|---|
| **A. 行動電源 (建議)** | USB 線接板子上方的 ST-LINK USB 孔 | 若插上後紅色電源燈 LD3 不亮、程式不跑：把 **JP1 短路** (UM1724：USB 充電器沒有 USB enumeration 時，需 JP1 ON 才會供電給 MCU) |
| B. 外部 5V | 5V 接 **CN7-6 (E5V)**、GND 接 CN7-8 | **JP5 改到 E5V** |
| C. 外部 7~12V | 接 **CN7-24 (VIN)**、GND 接 CN7-8 | **JP5 改到 E5V** |

- 用 B / C 之後要再用 USB 燒錄時，記得 JP5 改回 U5V (或兩者都接，先接外部電源再插 USB)。
- **不要從 CN7-18 (5V) 灌電進去**，那支是給 SD 模組用的 5V 輸出。

---

## 3. 操作

### 開機 → 自動開始紀錄
1. 插好 SD 卡，接上電源；ESP32 也接上電源 (足底感測器開機，ESP32 會自動掃描連線，約 5~15 秒)。
2. 約 3~5 秒後 (等 IMU 開機)，**LD2 綠燈開始慢閃 (1 秒亮滅一次) = 紀錄中**。
3. 每次開機會建立新檔，不會覆蓋舊檔。
4. 足底資料只要 ESP32 一送過來就會寫進 `runXXX_foot.csv`；ESP32 比較晚連上也沒關係，用時間戳對齊即可。

### 綠燈 LD2
| 狀態 | 意思 |
|---|---|
| 慢閃 (約 1 Hz) | 紀錄中 |
| 恆亮 | 已停止，可以安全拔卡 / 關電 |
| 快閃 (約 5 Hz) | SD 卡錯誤 (沒插、沒格式化、接觸不良)。插好後短按 B1 重試 |
| 不亮 | 開機中 (前 3 秒) |

### 藍色 B1 按鈕
| 操作 | 功能 |
|---|---|
| **短按** (紀錄中) | 停止紀錄、關檔、卸載 SD → 綠燈恆亮，可以拔卡 |
| **短按** (已停止) | 開一個新檔繼續紀錄 |
| **按住 3 秒** | 兩顆 encoder **歸零** (目前位置 = 0°)，按住時與放開後 1 秒請保持不動 |

### 關機 / 拔卡
- **先短按 B1，等綠燈恆亮再拔卡或關電**。
- 如果直接斷電：程式每秒會把資料寫回 SD 一次，最多遺失最後約 1 秒，檔案仍可讀。
  (此功能尚未在電腦上實際驗證，重要的實驗請一律先按 B1 再關電。)

### 歸零
- 歸零值存在 MCU 內部 flash，**斷電、重開機、重新燒錄都會保留**，不用每次開機都做。
- 只有要換新的零點時才需要：把關節擺到 0° 的位置，按住 B1 3 秒。
- 目前沒有燈號提示歸零完成；可在序列埠看到 `ZERO: enc1=… enc2=… saved=1`，或看 CSV 的 `encX_deg` 是否變成約 0。
- 歸零時取樣會暫停約 0.4 秒 (寫 flash)，CSV 會少約 40 行 (`t_s` 會跳一段)，這段時間的足底封包也會遺失。

---

## 4. 資料檔

每次紀錄產生兩個檔，放在 `/exoskeleton_project_record/enc_imu_foot/`：
- `run001.csv`：encoder + IMU，每 10 ms 一行 (100 Hz)
- `run001_foot.csv`：足底感測器，每收到一包 (左或右腳) 一行

### 每個裝置的 time code (都是 MCU 時鐘，單位秒，µs 解析度，可以直接相減)
| 欄位 | 檔案 | 代表的時間 |
|---|---|---|
| `t_s` | run | 這個週期開始的時間 |
| `enc_t_s` | run | encoder 取樣時間 (CS 拉 Low 的瞬間，AS5145 在這時鎖存角度) |
| `imu_t_s` | run | IMU 新封包出現的時間 (有 DRDY = DRDY 上升緣；沒有 DRDY = 輪詢到的時間，最多晚 1 ms)。這週期沒有新封包就沿用上一筆 |
| `imu_stf` | run | IMU **自己的時鐘** SampleTimeFine (0.1 ms)，用來檢查 IMU 跟 MCU 的時鐘漂移 |
| `t_rx_s` | foot | STM32 收到這包足底資料最後一個 byte 的時間 |
| `esp_us` | foot | **ESP32 自己的時鐘** (micros())，ESP32 收到 BLE 封包的時間，用來檢查 UART 延遲抖動 |

- 足底感測器本身沒有時鐘，最早能拿到的時間是 ESP32 收到 BLE 的 `esp_us`。BLE 本身的延遲 (感測器 → ESP32) 量不到。
- 兩個檔用 `footL_cnt` / `footR_cnt` (run) ↔ `seq` (foot)、`cycle` (foot) ↔ `cycle` (run) 互相對照。

### run001.csv 欄位
| 欄位 | 說明 |
|---|---|
| `cycle` | 週期編號 (連號，用來檢查有沒有漏) |
| `t_s` `enc_t_s` `imu_t_s` | 見上表 |
| `trig` | 2 = 輪詢到新 IMU 封包、1 = IMU DRDY、0 = Ticker / IMU 逾時 |
| `encX_raw` | encoder 原始值 0~4095 (未歸零) |
| `encX_deg` | **相對零點的角度 −180~+180°**；通訊錯誤時沿用上一筆 |
| `encX_stat` | 晶片狀態 bit (16 進位)：OCF COF LIN MagINC MagDEC Parity |
| `encX_err` | 0 = 正常。通訊錯誤：1 同位、2 OCF=0、4 COF=1、32 起始 bit；磁場警告：8 LIN、16 MagINC+MagDEC |
| `imu_new` / `imu_bad` | 這週期新的 IMU 封包數 (正常 1) / 錯誤封包數 |
| `imu_pc` / `imu_stf` | IMU PacketCounter (每行 +1) / SampleTimeFine (0.1 ms) |
| `roll` `pitch` `yaw` | 姿態角 (deg) |
| `gx` `gy` `gz` | 角速度 (deg/s) |
| `ax` `ay` `az` | 加速度 (m/s²)，靜止時大小約 9.8 |
| `footL_cnt` `footR_cnt` | 到這個週期為止收到的左 / 右腳封包數 (= foot 檔的 `seq`) |
| `busy_us` | 這週期讀感測器花的時間 (約 1.5 ms) |
| `dropped` | 累計丟掉的週期 (SD 太慢時才會增加，正常 0) |

### run001_foot.csv 欄位
| 欄位 | 說明 |
|---|---|
| `seq` | 該腳第幾包 (左右各自從 1 開始) |
| `foot` | `L` / `R` |
| `t_rx_s` | STM32 收到的時間 (見上表) |
| `cycle` | 收到時主檔最新的 `cycle` |
| `esp_us` | ESP32 收到 BLE 的時間 (µs，約 71 分鐘繞回一次)；舊版 ESP32 程式為空白 |
| `esp_seq` | ESP32 對該腳的封包編號 (0~65535)；跳號 = ESP32 → STM32 掉包 |
| `P1`~`P18` | 18 點壓力 (g) |
| `total_g` | 18 點加總 (g) |
| `cop_valid` `cop_x_mm` `cop_y_mm` | 壓力中心 (總重 < 50 g 時無效，X/Y 空白)；座標原點在程式 `FOOT_ORIGIN_X/Y` |

### 檢查時間對齊
```bash
python3 tools/check_time_align.py run001.csv --plot
```
會印出 (並畫成 `run001_time_align.png`)：
- encoder / IMU 的取樣間隔 (應該都約 10 ms)、cycle 跳號、IMU PacketCounter 跳號
- IMU 時鐘 vs MCU 時鐘的漂移 (ppm) 與 IMU 時間戳抖動
- 同一行 encoder 取樣 − IMU 封包時間 (應 < 1 ms)
- 左右腳的封包頻率、間隔、`esp_seq` 掉包數
- ESP32 時鐘 vs MCU 時鐘的漂移，以及 ESP32 → STM32 的延遲抖動
- 每包足底資料距離最近的 encoder 取樣差多少 (應在 ±5 ms 內)

只需要 `numpy`，`--plot` 另外需要 `matplotlib`。

---

## 5. 接電腦時 (選用)

序列埠 115200 baud (`screen /dev/ttyACM0 115200`)，每秒印一行狀態：
```
STAT rec=1 lines=1201 rate=100 imu_trig=100 enc_err=0/0 mag_warn=100/100 imu_miss=0 imu_bad=0 busy_max=1563us dropped=0
```
`rec` 是否在紀錄、`rate` 每秒週期數 (100)、`enc_err` 通訊錯誤 (應為 0)、`mag_warn` 磁場警告、`imu_miss` 沒讀到 IMU 的週期 (應為 0)。

接著一行足底狀態：
```
FOOT L=20 (gap_max=62ms) R=20 (gap_max=58ms) total L/R=1203/1198 foot_bad=0 foot_drop=0 foot_jit=850us
```
`L` / `R` 這一秒收到幾包 (0 = 那隻腳沒連上)、`gap_max` 最大間隔、`foot_bad` checksum 錯誤 (接線不良)、`foot_drop` STM32 佇列滿丟掉的 (正常 0)、`foot_jit` ESP32 → STM32 延遲抖動 (ESP32 沒送時間戳時顯示 `-`)。

| 指令 | 功能 |
|---|---|
| `g` / `x` | 開始 / 停止紀錄 (同 B1 短按) |
| `z` | encoder 歸零 (同 B1 按住 3 秒) |
| `s` / `S` | 即時資料輸出 10 Hz / 50 Hz (再按一次關閉) |
| `i` | 重新初始化 IMU (換 IMU 或接線後，不用重開機) |

即時圖表：`python3 ~/project/python/imu/live_plot_enc_imu.py` (開著時序列埠被佔用)。

### 編譯 / 燒錄
```bash
cd ~/project/mbed/enc_imu_sd_logger
mbed compile -m NUCLEO_F446RE -t GCC_ARM && openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program BUILD/NUCLEO_F446RE/GCC_ARM/enc_imu_sd_logger.bin verify reset exit 0x08000000"
```

---

## 6. 疑難排解

| 症狀 (序列埠) | 原因 / 處理 |
|---|---|
| 綠燈快閃、`SD init failed: -5005` | 沒偵測到 SD 卡：檢查 CN7-1~4、電源、卡有插好；插好後短按 B1 |
| `SD mount failed` | 卡不是 FAT32，用電腦格式化 |
| `IMU init: fw=00000000`，下一行 `diag … FFFFFFFF` | IMU 完全沒回應：IMU 沒電、CS/SCK/MISO 線斷 (今天換線後就好)、IMU 不在 SPI 模式 |
| `IMU init: fw=00000000`，`diag … 00000000` | MISO 被拉 Low：接錯腳或短路 |
| `enc_err` 不為 0 / encoder 讀值全 0 | 用 `encoder_chain_test` 逐顆測；PROG 要接地、檢查焊點 (今天 encoder 一顆重焊後就好) |
| encoder 角度跳動大、`mag_warn` 一直 100% | 磁場超出範圍：磁鐵要在**晶片那一面** (晶片在 PCB 底層)、對準中心、距離 0.5~1.5 mm |
| `dropped` 持續增加 | SD 卡太慢 / 接觸不良，換卡或改 `SD_SPI_FREQ` 為 1 MHz |
| `FOOT L=0 R=0` | ESP32 沒送資料：看 ESP32 序列埠是否 `LEFT/RIGHT connected: YES`；GPIO17 → CN10-33 (PA_10)、GND 有沒有共地 |
| `FOOT` 只有一隻腳 | 另一顆感測器沒開機或沒連上，重開 ESP32 讓它重新掃描 |
| `foot_bad` 一直增加 | UART 接觸不良，或 ESP32 與 STM32 鮑率不同 (`UART_BAUD` / `FOOT_BAUD`) |
| `foot_jit=-` | ESP32 是舊程式，重燒 `ESP32_sensor_test` (`SEND_TIMESTAMP = true`) |
