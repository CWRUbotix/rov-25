#!/bin/bash

# Add GZ_SIM_SYSTEM_PLUGIN_PATH to .bashrc only if it isn't already there
ROV_PATH=$(pwd)
if ! [[ $ROV_PATH == *rov-25 ]]
then
    echo "Not run from */rov-25 directory"
    echo "Run from $ROV_PATH"
    exit 1
fi

ROS_LINE="export GZ_SIM_RESOURCE_ROV_PATH=$ROV_PATH/src/surface/rov_gazebo/models:$ROV_PATH/src/surface/rov_gazebo/models/props:$ROV_PATH/src/surface/rov_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"
if ! grep -qF "$ROS_LINE" ~/.bashrc ;
    then echo "$ROS_LINE" >> ~/.bashrc ;
fi

. ~/.bashrc