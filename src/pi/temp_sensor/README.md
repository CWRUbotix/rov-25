# Flood Detection

## Overview

This package reads from the temperature sensor.

## Installation

```bash
sudo pip install bluerobotics-tsys01@git+https://github.com/bluerobotics/tsys01-python
```

## Usage

```bash
ros2 launch temp_sensor temp_sensor_launch.py
```

## Launch files

* **temp_sensor_launch.py:**  Primary launch file for temp sensor.

## Nodes

### temp_sensor

Reads and publishes temperature.

#### Published Topics

* **`/temperature`** ([rov_msgs/temperature])

    Temperature readings as floats


### test

Tests flooding.

[rov_msgs/temperature]: ../../rov_msgs/msg/Temperature.msg
