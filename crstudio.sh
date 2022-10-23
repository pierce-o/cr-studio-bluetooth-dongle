sudo modprobe cp210x
sudo echo 10c4 ea60 > /sys/bus/usb-serial/drivers/cp210x/new_id
sudo ./cp210x-cfg -m 10c4:ea60 -V 0x1A86 -P 0x55D3