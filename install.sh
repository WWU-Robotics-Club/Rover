# This script was tested on Ubuntu 18.04.3 64bit X86

# set up ROS melodic http://wiki.ros.org/melodic/Installation/Ubuntu
# Set up sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Make sure everything is updated
sudo apt -y update
# install ROS
sudo apt -y install ros-melodic-desktop-full
# Initialize rosdep
sudo rosdep init
rosdep update
# Add environment variables
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# Install dependancies
sudo apt -y install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Install some ROS packages
# Install ros-serial for arduino
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install ros-melodic-rosserial
# https://wiki.ros.org/rosbridge_suite
sudo apt-get install ros-melodic-rosbridge-server
# https://github.com/IntelRealSense/librealsense
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
# sudo apt-get install -y librealsense2-dbg # uncomment if debug symbols are needed

# Install PlatformIO
sudo apt-get install gcc python-pip && \
  pip install -U platformio && \
  platformio platform install teensy && \

# install various linters
python -m pip install cpplint
pip install pylint

# Install realsense sdk
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# Install Arduino IDE
ARDUINO_VER="arduino-1.8.10"
ARDUINO_PLATFORM="linux64"
# Download
wget https://downloads.arduino.cc/$ARDUINO_VER-$ARDUINO_PLATFORM.tar.xz
# Extract
sudo mkdir /opt/arduino
sudo tar xf $ARDUINO_VER-$ARDUINO_PLATFORM.tar.xz -C /opt/arduino
# Install
sudo /opt/arduino/$ARDUINO_VER/install.sh
# Cleanup
sudo rm $ARDUINO_VER-$ARDUINO_PLATFORM.tar.xz

# Add user to dialout group. Allows usb port access https://www.arduino.cc/en/Guide/Linux#toc6
sudo usermod -a -G dialout $USER
# reload dialout group so we don't have to logout and back in
# https://superuser.com/questions/272061/reload-a-linux-users-group-assignments-without-logging-out
CURRENT_GROUP="$(id -g)"
newgrp dialout
newgrp $CURRENT_GROUP

# Install Teensyduino arduino extension https://www.pjrc.com/teensy/td_download.html
# Allows non-root users to use it?
wget https://www.pjrc.com/teensy/49-teensy.rules
sudo cp 49-teensy.rules /etc/udev/rules.d/
# Download
wget https://www.pjrc.com/teensy/td_148/TeensyduinoInstall.linux64
# Install
echo "Click through the installer and set install location to /opt/arduino/$ARDUINO_VER"
chmod +x TeensyduinoInstall.linux64
sudo ./TeensyduinoInstall.linux64




