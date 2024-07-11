#!/bin/bash

sudo apt-get update
sudo apt-get install rapidjson-dev libignition-gazebo6-dev -y
# Can't be ssh clone because that breaks docker.
# Can either make a second file or just hope that our fork does not need lots of dev work
git clone -b fortress  https://github.com/CWRUbotix/rov-ardupilot-gazebo-fortress.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4

# Add IGN_GAZEBO_SYSTEM_PLUGIN_PATH to .bashrc only if it isn't already there
ROS_LINE='export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH'
if ! grep -qF "$ROS_LINE" ~/.bashrc ;
    then echo "$ROS_LINE" >> ~/.bashrc ;
fi

. ~/.bashrc