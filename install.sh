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

