sudo modprobe cp210x
sudo echo 1a86 55d3 > /sys/bus/usb-serial/drivers/cp210x/new_id
sudo ./cp210x-cfg -m 1a86:55d3 -V 0x10c4 -P 0xEA60