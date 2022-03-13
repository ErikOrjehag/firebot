
# Firebot!

## Connect

ubuntu@192.168.43.52
raspberry

## Fix display issue for China screen

Add to `/system-boot/usercfg.txt`:

```
hdmi_force_hotplug=1
hdmi_group=2
hdmi_mode=16
hdmi_drive=2
config_hdmi_boost=4
dtoverlay=vc4-fkms-v3d
```

https://raspberrypi.stackexchange.com/questions/107943/weird-display-on-vga-monitor-ubuntu-server-19-10-on-raspberry-pi-4

## Setup wifi

https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line

## Install raspi-config

sudo echo "deb http://archive.raspberrypi.org/debian/ buster main" >> /etc/apt/sources.list
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 7FA3303E
sudo apt-get update
sudo apt-get install raspi-config
sudo mount /dev/mmcblk0p1 /boot

Enable SSH and I2C

## Enable GPIO for non root user

sudo groupadd gpi
sudo adduser ubuntu gpio
sudo chown root.gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem

## Install pyaudio dependencies

sudo apt-get install libportaudio2 libportaudiocpp0 portaudio19-dev

## Install ROS2 Galagic

https://singleboardblog.com/install-ros2-on-raspberry-pi/

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
