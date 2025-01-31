# surface_main

## Overview

This package is for launching all of our surface code. It also includes the install script for ROS.

## Usage

Run the main node with

```bash
ros2 launch surface_main surface_all_nodes_launch.py
```

or run just operator or just pilot nodes with

```bash
ros2 launch surface_main surface_operator_launch.py
```

```bash
ros2 launch surface_main surface_pilot_launch.py
```

## Launch files

* **surface_all_nodes_launch.py:** Launches all surface nodes. ex. `ros2 launch surface_main surface_all_nodes_launch.py controller_profile:=1`

* **surface_operator_launch.py:** Launches operator gui and all related nodes.

* **surface_pilot_launch.py:** Launches pilot gui and all related nodes.
