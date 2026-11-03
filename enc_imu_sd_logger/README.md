# enc_imu_sd_logger 操作手冊

2 顆 AS5145 編碼器 (串接) + 1 顆 Xsens MTi IMU，以 100 Hz 同步讀取並存到 microSD 卡。
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

---

## 2. 電源 (不接電腦獨立運作)

整套 (板子 + 2 encoder + IMU + SD) 約 150~300 mA。擇一：

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
1. 插好 SD 卡，接上電源。
2. 約 3~5 秒後 (等 IMU 開機)，**LD2 綠燈開始慢閃 (1 秒亮滅一次) = 紀錄中**。
3. 每次開機會建立新檔，不會覆蓋舊檔。

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
- 歸零時取樣會暫停約 0.4 秒 (寫 flash)，CSV 會少約 40 行 (`t_s` 會跳一段)。

---

## 4. 資料檔

路徑：`/exoskeleton_project_record/enc_imu/run001.csv`、`run002.csv`…
每 10 ms 一行 (100 Hz)，第一行是欄位名稱：

| 欄位 | 說明 |
|---|---|
| `cycle` | 週期編號 (連號，用來檢查有沒有漏) |
| `t_s` | MCU 時間 (秒，µs 精度) |
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
| `busy_us` | 這週期讀感測器花的時間 (約 1.5 ms) |
| `dropped` | 累計丟掉的週期 (SD 太慢時才會增加，正常 0) |

---

## 5. 接電腦時 (選用)

序列埠 115200 baud (`screen /dev/ttyACM0 115200`)，每秒印一行狀態：
```
STAT rec=1 lines=1201 rate=100 imu_trig=100 enc_err=0/0 mag_warn=100/100 imu_miss=0 imu_bad=0 busy_max=1563us dropped=0
```
`rec` 是否在紀錄、`rate` 每秒週期數 (100)、`enc_err` 通訊錯誤 (應為 0)、`mag_warn` 磁場警告、`imu_miss` 沒讀到 IMU 的週期 (應為 0)。

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
