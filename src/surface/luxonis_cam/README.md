# Luxonis Cam

## Overview

This package uploads a Luxonis pipeline to our Luxonis dual cam, and provides a ROS node which handles starting/stopping unrectified, rectified, and disparity/depth map streams from it.

## Usage

The luxonis_cam_driver node should be launched automatically with the pilot GUI. To run it separately, you can run

```bash
ros2 launch luxonis_cam luxonis_launch.py
```

## Launch files

* **luxonis_launch.py:** launches the luxonis_cam_driver node (cam_driver.py)

## Nodes

### luxonis_cam_driver

Listens to CameraManage to toggle streams and publishes currently running streams.

#### Published Topics

No stereo computations required (computationaly cheap for Luxonis cam's onboard compute):

* **`/surface/lux_raw/image_raw`** ([sensor_msgs/msg/Image])

    Raw, unrectified Luxonis streams. Toggles between left & right eyes.

Stereo computations required (computationaly expensive for Luxonis cam's onboard compute):

* **`/surface/rect_left/image_raw`** ([sensor_msgs/msg/Image])

    Rectified left eye Luxonis stream.

* **`/surface/rect_right/image_raw`** ([sensor_msgs/msg/Image])

    Rectified right eye Luxonis stream.

* **`/surface/disparity/image_raw`** ([sensor_msgs/msg/Image])

    Disparity map Luxonis stream.

* **`/surface/depth/image_raw`** ([sensor_msgs/msg/Image])

    Depth map Luxonis stream.

#### Services

* **`/surface/manage_luxonis`** ([rov_msgs/srv/CameraManage.srv])

    Toggle LUX_ streams. Enabling/disabling stereo-dependent streams will enable/disable onboard stereo computation.


## Calibration

From [this guide](https://docs.luxonis.com/hardware/platform/depth/calibration).

Install depthai repo (installed to `~/depthai` on competition laptop):

```bash
git clone https://github.com/luxonis/depthai.git
cd depthai
git submodule update --init --recursive

python3 -m venv venv
. venv/bin/activate

python3 install_requirements.py
```

Run the calibration script (`-s` is square size in cm, `-nx` & `-ny` are grid dimensions in # squares) to take pictures:

```bash
python3 ~/depthai/calibrate.py -s 2.208 -nx 12 -ny 9 -brd ~/rov-25/src/surface/luxonis_cam/calibration/rov_depth_enclosure.json
```

> Optionally get on the `develop` branch if you get errors with our cams (`OV9782` RGB, not `OV9282` mono) being detected incorrectly:
>
> ```
> git checkout develop
> ```

Images will be saved to `depthai/dataset`. Press `s` to continue to calibration. Calibration results are stored in `depthai/resources`.

Optionally, rerun calibration in the images in `dataset`:

```bash
python3 ~/depthai/calibrate.py -s 2.208 -nx 12 -ny 9 -brd ~/rov-25/src/surface/luxonis_cam/calibration/rov_depth_enclosure.json -m process
```

[sensor_msgs/msg/Image]: <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html>