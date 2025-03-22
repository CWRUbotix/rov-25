# Flight Control

## Overview

This package includes keyboard and PS5 controller nodes.

## Usage

The PS5 controller (`manual_control_node`) is run when the pilot is launched.
You can run them on their own with:

```bash
ros2 launch flight_control flight_control_launch.py
```

The keyboard controller (`keyboard_control_node`) is not run normally.
You can run it with:

```bash
ros2 launch flight_control keyboard_control_launch.py
```

## Launch files

* **flight_control_launch.py:** launches the PS5 controller

* **keyboard_control_launch.py:** launches the keyboard controller under the `/surface` namespace

## Nodes

### manual_control_node

Controls motors, manipulators, and camera switching (if applicable) from the PS5 controller.

#### Subscribed Topics

* **`/surface/joy`** ([sensor_msgs/msg/Joy])

    PS5 controller instructions.

* **`/surface/camera_switch`** ([rov_msgs/msg/CameraControllerSwitch])

    Instructions to change which camera should be active. TODO: Remove this if possible after upgrading to FLIR cams.

#### Published Topics

* **`/tether/manipulator_control`** ([rov_msgs/msg/Manip])

    Manipulator instructions for the Pi.

* **`/tether/mavros/manual_control/send`** ([mavros_msgs/msg/ManualControl])

    The movement instructions for the Pixhawk

#### Services

* **`/tether/mavros/cmd/command`** ([mavros_msgs/srv/CommandLong.srv])

    Instructions to set the Valve Manipulator servo

* **`/tether/mavros/cmd/arming`** ([mavros_msgs/srv/CommandBool.srv])

    Instruction to arm the robot

### keyboard_control_node

Controls motors (only) from the keyboard. Not run by general surface launch files. Run this separately with its launch file to use it.
This node can publish concurrently with manual control/auto docking.

#### Published Topics

* **`/tether/mavros/manual_control/send`** ([mavros_msgs/msg/ManualControl])

    The movement instructions for the Pixhawk
