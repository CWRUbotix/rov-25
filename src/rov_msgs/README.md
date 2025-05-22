# rov_msgs

## Overview

This is the package for all our msg, srv, and action definitions.

## Msg files

[CameraControllerSwitch.msg](msg/CameraControllerSwitch.msg)

Used for toggling through camera feeds in the pilot gui.

[Manip.msg](msg/Manip.msg)

Used for toggling manipulators on and off.

## Srv files

[AutonomousFlight.srv](srv/AutonomousFlight.srv)

Send to a automation task's control topic to start or stop that automation task

[GeneratePhotosphere.srv](srv/GeneratePhotosphere.srv)

Send a request to generate a photosphere and receive whether it was generated properly
