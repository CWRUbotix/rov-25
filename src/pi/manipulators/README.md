# manipulators

## Overview

This package is used to toggle manipulators plugged into the I2C bus on the motherboard. The chip we communicate with is the TCA9555 more documentation can be found [here](https://www.ti.com/lit/gpn/tca9555).

## Installation

## Usage

Run the main node with

```bash
ros2 run manipulators manipulators
```

Run the test node with

```bash
ros2 run manipulators test
```

## Launch files

* **manip_launch.py:** Launches the manipulators node.

## Nodes

### manipulator

Toggle a manipulator on or off on receiving a msg.

#### Subscribed Topics

* **`/manipulator_control`** ([msg/Manip])

    The control message for activating manipulators. Set `manip_id` to anything other than "valve" to control the claw manips.


### valve_manipulator

Toggle the servo on or off on receiving a message.

#### Subscribed Topics

* **`/manipulator_control`** ([msg/Manip])

    The control message for activating manipulators. Set `manip_id` to "valve" to control the valve manip.

### test

Sends on and off signals to the IC2 board to confirm it is working. There is no ROS features being used.

[msg/Manip]:../../rov_msgs/msg/Manip.msg
