# rov_flir

## Overview

This is a boilerplate package which launches the camera_driver_node from the flir_launch package.

## Setup

Add the following line to `.bashrc`

```bash
SPINNAKER_GENTL64_CTI=/opt/ros/${ROS_DISTRO}/lib/spinnaker-gentl/Spinnaker_GenTL.cti
```

Run the following spinnaker_camera_driver setup from a regular terminal. Don't run it in the VSCode terminal (it doesn't work there for some reason).

```bash
ros2 run spinnaker_camera_driver linux_setup_flir
```

## Debugging

You can install the Spinnaker SDK & Spinview GUI for debugging [here](https://www.flir.com/products/spinnaker-sdk/).

If you get errors about cameras on the wrong subnet, install this and open Spinview. Write click the entries for each camera and select "Auto Force IPs".

## Usage

Run the main node with

```bash
ros2 launch rov_flir flir_launch.py
```

## Settings

In the config folder are the nodes from the spinnaker api, these are different from ROS nodes. They are basically settings of the cameras. Each setting in the config file has a `name` which is the name of the ROS parameter, a `type` which is the data type of the field, and finally each setting has a `node` which is the name of the setting in spinnaker.

## Launch files

* **flir_launch.py:** Launches the camera_driver_node for the two flir cameras.

## Published Topics

* **`/front_cam/image_raw`** ([sensor_msgs/msg/Image])

    Image feed for front camera.

* **`/front_cam/meta`** ([flir_camera_msgs/msg/ImageMetaData])

    Front camera image meta data.

* **`/front_cam/camera_info`** ([sensor_msgs/msg/CameraInfo])

    Front camera info.

* **`/bottom_cam/image_raw`** ([sensor_msgs/msg/Image])

    Image feed for bottom camera.

* **`/bottom_cam/meta`** ([flir_camera_msgs/msg/ImageMetaData])

    Bottom camera image meta data.

* **`/bottom_cam/camera_info`** ([sensor_msgs/msg/CameraInfo])

    Bottom camera info.

## Subscribed Topics

* **`/front_cam/control`** ([flir_camera_msgs/msg/CameraControl])
    Controls exposure and gain.

* **`/bottom_cam/control`** ([flir_camera_msgs/msg/CameraControl])
    Controls exposure and gain.

[sensor_msgs/msg/Image]: https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html
[flir_camera_msgs/msg/ImageMetaData]: https://github.com/ros-drivers/flir_camera_driver/blob/humble-devel/flir_camera_msgs/msg/ImageMetaData.msg
[sensor_msgs/msg/CameraInfo]: https://docs.ros2.org/latest/api/sensor_msgs/msg/CameraInfo.html
[flir_camera_msgs/msg/CameraControl]: https://github.com/ros-drivers/flir_camera_driver/blob/humble-devel/flir_camera_msgs/msg/CameraControl.msg