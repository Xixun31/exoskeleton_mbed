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
