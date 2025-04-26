# photosphere

## Overview

This package is used to stream the images from the fisheye cameras and to create an equirectangular projection from them.

## Usage

Run the main node with

```bash
ros2 launch phosotphere photosphere_launch.py
```

## Launch files

* **photosphere_launch.py:** launches the Photosphere Node

## Nodes

### photosphere

Creates a photosphere from fisheye images.

#### Subscribed Topics

* **`/transceiver_control`** ([msg/FloatCommand.msg])

    When to submerge the float and what time it was submerged.

#### Published Topics

* **`/transceiver_data`** ([msg/FloatCommand.msg])

    The data received from the float.


[msg/FloatCommand.msg]:../../rov_msgs/msg/FloatCommand.msg