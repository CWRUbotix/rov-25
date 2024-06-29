# Flood Detection

## Overview

This package launches flood detection.

## Installation

```bash
sudo apt-get install python3-lgpio
```

## Usage

```bash
ros2 launch flood_detection flood_detection_launch.py
```

## Launch files

* **flood_detection_launch.py:**  Primary launch file for flood detection.

## Nodes

### flood_detector

Detects flooding.

#### Published Topics

* **`/flooding`** ([rov_msgs/flooding])

    Whether rov is flooding.


### test

Tests flooding.

[rov_msgs/flooding]: ../../rov_msgs/msg/Flooding.msg
