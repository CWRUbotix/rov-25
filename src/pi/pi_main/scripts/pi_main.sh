#!/bin/bash

source /home/rov/rov-25/install/setup.bash
ros2 launch pi_main pi_launch.py

docker start blueos-bootstrap
