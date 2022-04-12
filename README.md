
# Firebot!

# Connect

ubuntu@192.168.43.52
raspberry

# Setup Ubuntu Focal 20.04 on Rasperry Pi 3B+

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

```
sudo echo "deb http://archive.raspberrypi.org/debian/ buster main" >> /etc/apt/sources.list
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 7FA3303E
sudo apt-get update
sudo apt-get install raspi-config
sudo mount /dev/mmcblk0p1 /boot
```

Then `raspi-config` and enable SSH and I2C

## Enable GPIO and I2C for non root user

```
sudo groupadd gpi
sudo groupadd i2c
sudo adduser ubuntu gpio
sudo adduser ubuntu i2c
sudo nano /etc/udev/rules.d/99-my.rules

RUN+="/bin/sh -c 'chown root.gpio /dev/gpiomem && chmod g+rw /dev/gpiomem'"
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0660"

sudo reboot
```

## Install pyaudio dependencies

This is needed to pip install pyaudio:

`sudo apt-get install libportaudio2 libportaudiocpp0 portaudio19-dev`

## Increase i2c speed to 400kHz

`sudo nano /boot/firmware/config.txt` and find `dtparam=i2c_arm=on` modify to say `dtparam=i2c_arm=on,i2c_arm_baudrate=400000` instead. Reboot then `./i2cspeed.sh` to verify the change.

## Install ROS2 Galagic

https://singleboardblog.com/install-ros2-on-raspberry-pi/

```
sudo apt update
sudo apt install curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt install ros-galactic-ros-base -y

echo "source /opt/ros/galactic/setup.bash" >>~/.bashrc
exec "$SHELL"

pip3 install colcon-common-extensions
```

## Mouse teleop

```
ros2 run mouse_teleop mouse_teleop --ros-args --remap /mouse_vel:=/cmd_vel
```
