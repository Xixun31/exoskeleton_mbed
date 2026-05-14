ln -s ../mbed-os .

cp ../imu_uart/mbed_app.json .
cp ../imu_uart/.mbed .

mbed compile -m NUCLEO_F446RE -t GCC_ARM -f

cp -X BUILD/NUCLEO_F446RE/GCC_ARM/XXX.bin /Volumes/NOD_F446RE/

ls /dev/cu.usb*

screen /dev/cu.usbmodem103 115200