#!/bin/bash

# Upgrade first
sudo apt-get update
sudo apt-get upgrade -y

# Ubuntu Universe install
sudo apt-get install software-properties-common -y
sudo add-apt-repository universe -y

# Adding ROS 2 repo to system
sudo apt-get update
sudo apt-get install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb"
sudo apt install /tmp/ros2-apt-source.deb

# Install pip
sudo apt-get install python3-pip -y

# Update any installed packages
sudo apt-get update
sudo apt-get upgrade -y

# Install dev tools like rosdep among others. This is ros-distro agnostic.
# More info can be found here
# https://discourse.ros.org/t/ros-developer-tools-now-in-binary-form/29802
sudo apt-get install ros-dev-tools -y

# Install ROS2
sudo apt-get install ros-jazzy-desktop -y
sudo apt-get upgrade -y

# Add setup.bash to .bashrc only if it isn't already there
ROS_LINE='source /opt/ros/jazzy/setup.bash'
if ! grep -qF "$ROS_LINE" ~/.bashrc ;
    then echo "$ROS_LINE" >> ~/.bashrc ;
fi

# Done to suppress setup.py install deprecated warnings
# Can be removed once ROS redoes their python build system
PYTHON_WARNINGS_LINE='PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources; export PYTHONWARNINGS'
if ! grep -qF "$PYTHON_WARNINGS_LINE" ~/.bashrc ;
    then echo "$PYTHON_WARNINGS_LINE" >> ~/.bashrc ;
fi

source ~/.bashrc

# Start rosdep
sudo rosdep init
rosdep update
source ~/.bashrc