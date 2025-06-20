# gui

## Overview

This package is for all the code related go the driver station GUI. The GUI is comprised of our custom widgets found in the `gui/widgets` folder.
`gui_node.GUINode` is initialized for each GUI, and includes helper methods to interact with the ROS network from the GUI.
`app.py` is the a custom superclass which inherits from a PyQT QWidget.
Then `operator_app.py` and `pilot_app.py` inherit from App to make our two custom GUIs.

## Installation

Install pypdf with

```bash
pip install pypdf --break-system-packages
```

## Usage

Run the operator GUI with

```bash
ros2 launch gui operator_launch.py
```

Run the pilot GUI with

```bash
ros2 launch gui pilot_launch.py
```

## Launch files

* **operator_launch.py:** Launches the operator GUI, ex. `ros2 launch gui operator_launch.py theme:=watermelon`

  * **`theme`** : Theme used by the GUI; options are `dark`, `light`, `watermelon`. Default: `dark`.

* **pilot_launch.py:** Launches the pilot GUI, ex. `ros2 launch gui pilot_launch.py theme:=watermelon gui:=debug`

  * **`theme`** : Theme used by the GUI; options are `dark`, `light`, `watermelon`. Default: `dark`.

  * **`gui`** : Whether to use vertical or debug GUI; options are `pilot`, `debug`. Default: `pilot`.

## GUI Node
Singleton ROS Node subclass which has helper methods for creating publishers, subscriptions, clients, and services for each GUI.

## Widgets

### Arm

Has two buttons for arming and disarming the pixhawk.

#### Subscribed Topics

* **`/surface/vehicle_state_event`** ([rov_msgs/msg/VehicleState])

    Indicates whether the vehicle is connected and whether it is armed.

#### Services

* **`/surface/arming`** ([rov_msgs/srv/VehicleArming])

    Sends a request to arm or disarm the flight computer. Receives a confirmation about the success of the arm or disarm.

### Flood Warning

Shows whether the robot is flooding or not on the GUI

#### Subscribed Topics

* **`/tether/flooding`** ([rov_msgs/msg/Flooding])

    A custom message for whether the robot is actively flooding

### Logger

Reads ROS logging information and displays it on the gui.

#### Subscribed Topics

* **`/rosout`** ([rcl_interfaces/msg/Log])

    The standard /rosout for communication information to the user.

### Timer

A simple start and stop timer that counts down from 15 minutes.

### Video Widget

A widget to display video. There are two subclasses of the video widget: `PauseableVideoWidget` adds pausing the video stream and `SwitchableVideoWidget` enables toggling between video feeds.

#### Subscribed Topics

* **`/front_cam/image_raw`** ([sensor_msgs/msg/Image])

    The video feed from the front of the robot.

* **`/bottom_cam/image_raw`** ([sensor_msgs/msg/Image])

    The video feed from the bottom of the robot.

* **`/camera/color/image_raw`** ([sensor_msgs/msg/Image])

    The video feed from the intel realsense.

[rov_msgs/msg/Flooding]: ../../rov_msgs/msg/Flooding.msg
[rcl_interfaces/msg/Log]: https://github.com/ros2/rcl_interfaces/blob/rolling/rcl_interfaces/msg/Log.msg
[sensor_msgs/msg/Image]: <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html>
