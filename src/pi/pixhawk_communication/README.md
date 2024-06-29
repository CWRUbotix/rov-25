# Pixhawk Communication

## Overview

Our Pixhawk is flashed in a software named [QGroundControl](http://qgroundcontrol.com/).

In the past we have flashed in Windows because Linux recently has been acting strange.

The software we flash onto the pixhawk is called [ArduSub](https://www.ardusub.com/).

Currently we have [Vectored ROV w/ Four Vertical Thrusters](https://www.ardusub.com/quick-start/vehicle-frame.html) vehicle frame.

We communicate to the Pixhawk by using [mavros](https://github.com/mavlink/mavros) which uses [mavlink](https://mavlink.io/en/).

To set up a Pixhawk USB port use [this](https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html) guide.

To remotely use QGroundControl ssh into the pi and run the following.

```bash
mavproxy.py --out {Your IP adress}:14550
```

If you get this error:

```
LSM303D _read_data_transaction_accel: _reg7_expected unexpected
```

Try uninstalling and reinstalling the package:
```bash
pip uninstall mavproxy
pip install mavproxy
```

## Installation

Install geographiclib dependencies.

```bash
sudo su -c "bash <(wget -qO- https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh)" root
```

## Usage

Run the main node with

```bash
ros2 launch pixhawk_communication mavros_launch.py
```

## Launch files

* **mavros_launch.py:** Launches the mavros nodes need for communicating with the pixhawk.

## Communication

### Topics

* **`'/mavros/rc/override'`** ([mavros_msgs/msg/OverrideRCIn])

    Used for telling the rov which direction to travel. This [page](https://www.ardusub.com/developers/rc-input-and-output.html) shows which channels is which direction.

### Services

* **`"/mavros/cmd/ඞrming"`** ([mavros_msgs/srv/CommandBool])

    Used for arming and disarming the rov.

[mavros_msgs/srv/CommandBool]: https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/srv/CommandBool.srv
[mavros_msgs/msg/OverrideRCIn]: https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/OverrideRCIn.msg
