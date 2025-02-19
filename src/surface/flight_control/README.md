# Flight Control

## Overview

This package includes keyboard, PS5 controller, and automatic docking pilot nodes. It abstracts creating motor control messages with `PixhawkInstruction`s.

## Usage

The PS5 controller (`manual_control_node`) and the auto docking controller (`auto_docking_node`) are both run when the pilot is launched.
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

* **flight_control_launch.py:** launches the PS5 controller and readies the auto docking controller

  * **`controller_mode`** Whether the controller arms or toggles cameras; options are 0 (arming), 1 (cameras). Default: 0.

  * **`controller_profile`** Profile controls what buttons do what; value is the profile index. Default: 0.

* **keyboard_control_launch.py:** launches the keyboard controller under the `/surface` namespace

## Nodes

### manual_control_node

Controls motors, manipulators, and camera switching (if applicable) from the PS5 controller.

#### Subscribed Topics

* **`/surface/joy`** ([sensor_msgs/msg/Joy])

    PS5 controller instructions.

#### Published Topics

* **`/tether/mavros/rc/override`** ([mavros_msgs/msg/OverrideRcIn])

    The movement instructions for the Pixhawk.

* **`/tether/manipulator_control`** ([rov_msgs/msg/Manip])

    Manipulator instructions for the Pi.

* **`/surface/camera_switch`** ([rov_msgs/msg/CameraControllerSwitch])

    Instructions to change which camera should be active. TODO: Remove this if possible after upgrading to FLIR cams.

### keyboard_control_node

Controls motors (only) from the keyboard. Not run by general surface launch files. Run this separately with its launch file to use it.
This node can publish concurrently with manual control/auto docking.

#### Published Topics

* **`/tether/mavros/rc/override`** ([mavros_msgs/msg/OverrideRcIn])

    The movement instructions for the Pixhawk. This node can publish concurrently with manual control.

### auto_docking_node

Execute an automatic docking sequence. This node must be "activated" with its service before it will publish movement instructions.
Once activated, it will publish concurrently with manual control/keyboard control nodes.

TODO: actually implement autodocking.

#### Services

* **`/surface/auto_docker_control`** ([rov_msgs/srv/AutonomousFlight.srv])

    Toggles whether the auto docker is running (sending instructions) and returns the new state.
