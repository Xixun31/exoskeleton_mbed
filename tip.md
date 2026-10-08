ln -s ../mbed-os .

cp ../imu_uart/mbed_app.json .
cp ../imu_uart/.mbed .

ls /dev/cu.usb*

screen /dev/cu.usbmodem103 115200

# mac
mbed compile -m NUCLEO_F446RE -t GCC_ARM -f

cp -X BUILD/NUCLEO_F446RE/GCC_ARM/XXX.bin /Volumes/NOD_F446RE/

# ubuntu
mbed compile -m NUCLEO_F446RE -t GCC_ARM

# 燒錄方式 1：使用 openocd 燒錄 (最穩定，推薦)
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program BUILD/NUCLEO_F446RE/GCC_ARM/imu_uart.bin verify reset exit 0x08000000"

# 燒錄方式 2：複製到虛擬隨身碟 (若有掛載隨身碟)
cp ./BUILD/NUCLEO_F446RE/GCC_ARM/imu_uart.bin /media/xixun/NOD_F446RE/ && sync


# ===== Ubuntu 完整流程 (以 encoder 為例) =====
# 專案名稱 = 資料夾名稱，.bin 檔名也會跟資料夾同名 (encoder -> encoder.bin)

# 1. 進入專案資料夾 (注意是 cd，前面不要多打 ~)
cd /home/xixun/project/mbed/encoder

# 2. 編譯 (加 -c 可以清除快取重新編譯)
mbed compile -m NUCLEO_F446RE -t GCC_ARM

# 3. 燒錄 (看到 ** Verified OK ** 就是成功)
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program BUILD/NUCLEO_F446RE/GCC_ARM/encoder.bin verify reset exit 0x08000000"

# 編譯 + 燒錄一行完成 (編譯失敗就不會燒錄)
mbed compile -m NUCLEO_F446RE -t GCC_ARM && openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program BUILD/NUCLEO_F446RE/GCC_ARM/encoder.bin verify reset exit 0x08000000"

# 只重置板子、不燒錄 (讓程式從頭跑)
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init; reset run; exit"

# 4. 看序列埠輸出
ls /dev/ttyACM*
screen /dev/ttyACM0 115200
# 離開 screen：Ctrl+A 然後按 K，再按 Y

# 如果出現 Device or resource busy：代表 port 被別的程式 (通常是 screen) 占用
fuser -v /dev/ttyACM0      # 查是哪個程式占用
kill <PID>                 # 關掉它

# 換別的專案時，把上面的 encoder 換成該資料夾名稱即可，例如 encoder_speed_test
