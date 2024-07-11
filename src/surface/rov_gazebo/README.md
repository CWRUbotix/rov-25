# rov_gazebo

## Overview

Runs the simulation as well as stores all the description files.

## Installation

Run the `[SETUP] Simulation` Task.

## Usage

Run the simulation and GUI with

```bash
ros2 launch rov_gazebo sim_launch.py
```

## Config files

`config/`

* **pixhawk_dumps.params** A copy of the params from the pixhawk for reference

* **pixhawk_dumps.params** Our modified params for the simulated pixhawk

## Launch files

* **sim_launch.py:** Runs the simulation.
