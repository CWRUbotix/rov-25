#!/bin/bash

# Add setup.bash to .bashrc only if it isn't already there
ROS_LINE="source $(pwd)/install/setup.bash"
if ! grep -qF "$ROS_LINE" ~/.bashrc ;
    then echo "$ROS_LINE" >> ~/.bashrc ;
fi