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
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get upgrade -y

# Install dev tools like rosdep among others. This is ros-distro agnostic.
# More info can be found here
# https://discourse.ros.org/t/ros-developer-tools-now-in-binary-form/29802
sudo apt-get install ros-dev-tools -y

# Install pip
sudo apt-get install python3-pip -y

# Install ROS2
sudo apt-get install ros-jazzy-ros-base -y
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

# Install only Pi dependencies
# Installs ROS dependencies
source /opt/ros/jazzy/setup.sh && export PIP_BREAK_SYSTEM_PACKAGES=1 && rosdep install --from-paths src/pi --ignore-src -r -y

# Add setup.bash to .bashrc
source "$(pwd)/.vscode/rov_setup.sh"
