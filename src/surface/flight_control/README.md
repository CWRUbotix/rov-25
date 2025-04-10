# Flight Control

## Overview

This package create a Mavlink connection to a flight computer on the vehicle running Ardusub and sends it control messages. Control messages are derived from the pilot's joystick inputs. This node also read's the vehicles Mavlink heartbeat messages to determine its status and publishes status information to other nodes. All of these functions are contained in the monolithic `mavlink_manual_control` node. Previously, functionality was divided into many small nodes, but consolidating in a single process (and using pygame's joystick library) significantly reduced the control latency compared to our previous solution.

## Usage

The mavlink manual control node should be launched automatically with the operator gui. To run it separately, you can run

```bash
ros2 launch flight_control flight_control_launch.py
```


## Launch files

* **flight_control_launch.py:** launches the PS5 controller and readies the auto docking controller

  * **`controller_profile`** Profile controls what buttons do what; value is the profile index. Default: 0.

* **keyboard_control_launch.py:** DEPRECATED! Launches the keyboard controller under the `/surface` namespace

## Nodes

### mavlink_manual_control

Reads joystick input and communicates with the vehicle

#### Subscribed Topics

* **`/tether/pi_heartbeat`** ([rov_msgs/msg/Heartbeat])

    Indicates that the ROS connection to the pi is alive.

#### Published Topics

* **`/surface/vehicle_state_event`** ([rov_msgs/msg/vehicle_state_event])

    The status of the various vehicle systems.

#### Services

* **`/surface/arming`** ([rov_msgs/srv/Arming.srv])

    Arm or disarm the flight computer on the vehicle. Arming the vehicle is necessary to spin thrusters.
