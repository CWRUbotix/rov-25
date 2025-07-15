# Pi Main

## Overview

This package launches the rest of the Pi packages. It should be run on Pi boot up.

## Setup

### Flashing

Flashing is the process of installing BlueOS on a Pi. This will overwrite anything currently on the Pi. To do so, download the BlueOS image from BlueRobotics' [website](https://blueos.cloud/docs/stable/usage/installation/). Choose the ARMv8 (64 bit) image.

We use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) for flashing, though similar tools like Balena Etcher also work. To flash a Pi compute module's internal EMMC storage (rather than an SD card), follow [this](https://www.jeffgeerling.com/blog/2020/how-flash-raspberry-pi-os-compute-module-4-emmc-usbboot) tutorial. In Raspberry Pi Imager, select the custom image downloaded from BlueRobotics. To flash an SD card instead, you can skip the tutorial and use Raspberry Pi Imager to directly flash the custom image to an SD card connected to your computer.


### Setup Laptop for Direct Ethernet Access
BlueOS expects to have an IP address of 192.168.2.2, and it expects the control station laptop to have the address 192.168.2.1. If you directly connect the Pi to your laptop with an ethernet cable, you need to configure your laptop to have the correct IP address. This is NOT required if the Pi is connected through a router.

#### Linux

1. Open Advanced Network Configuration applet

2. Select the network under Ethernet

3. Under IPv4 Settings set the Method `Shared to other computers`

4. Set the Address to 192.168.2.1 and the Netmask to 24

#### Windows

1. On Windows, search "view network connections" in the search bar, this should bring you to the control panel.

    ![Screenshot of the control panel Network Connections page showing a wifi and an ethernet adapter](images/1-control-panel.png)

2. Right click on your wifi adapter and click on "Properties".

    ![Screenshot of the right click menu of a wifi adapter with "Properties" underlined](images/2-wifi-properties-button.png)

3. Go to the "sharing" tab.

    ![Wifi Properties window with the sharing tab underlined](images/3-wifi-sharing.png)

4. Check both checkboxes, and accept any popups. If there is a dropdown menu, select your ethernet adapter. Press "OK".

    ![Wifi Properties window both checkboxes checked](images/4-wifi-sharing-checkboxes.png)

5. Now right click on your ethernet adapter and choose "Properties".

    ![Screenshot of the right click menu of an ethernet adapter with "Properties" underlined](images/5-ethernet-properties-button.png)

6. Double click on "Internet Protocol Version 4".

    ![Screenshot of the ethernet properties window with "Internet Protocol Version 4" highlighted](images/6-ethernet-properties-items.png)

7. Set the IP address to "192.168.2.1". Set the subnet mask to "255.255.255.0". **The IP address in this image is incorrect.**

    ![Screenshot of the ipv4 properties window showing the IP address and subnet mask](images/7-ipv4-properties.png)


### Setting up a Router
If you're connecting the Pi to your laptop through a router, the router should be configured to assign the expected IP addresses to both the Pi and your laptop. Routers should have some way to configure them to assign a static IP to a particular device, based on its MAC address. You can find a linux device's MAC address via the `ip a` command, or the router should be able to tell you the MAC address of connected devices. You want to assign the Pi the address 192.168.2.2 and the laptop 192.168.2.1. In order to assign an address in the 192.168.2 block, you may need to change the router's subnet mask to 255.255.240.0. After configuring, reboot the router. The Pi and laptop should be assigned the appropriate addresses.


### Configure BlueOS
- Power on the pi and set up the network per one of the two previous sections
- While connected to the pi via ethernet, visit the BlueOS web dashboard (via a browser) at http://192.168.2.2
- There should be a popup for the setup wizard. Otherwise, you can access it in settings (gear icon in the bottom left).
    - Sometimes I've found that the Pi needs to be rebooted for the setup manager to work
    - Choose ROV Setup.
    - Set the name of the vehicle to whatever you think it should be called (just for fun)
    - Under "Parameter Sets", choose "Heavy BlueROV2"
    - The wizard should download and install a stable version of ArduSub. Exit the wizard when it's done.
- Enable pirate mode by clicking on the skull and crossbones in the top right (this shows advanced options).


### Install the CWRUbotix Extension
The CWRUbotix extension runs our custom code in the pi packages (including this one). To install it, navigate to the Extensions tab, click "Installed" on the top, and press the plus button on the lower right. Fill out the options in the create extension form like so:
- Extension Identifier: Used for internal identification only. I recommend `cwrubotix.ros2` or similar.
- Extension Name: Human-readable name for the extension. Currently "CWRUbotix ROS" but feel free to change.
- Docker Image: URI for the docker image. Currently `noahmollerstuen/blueos-cwrubotix-ros2-extension` (hopefully will be moved to a team docker account)
- Docker Tag: Docker images can have multiple versions (like branches). By default, use `main`.
- Original settings: The permissions and other metadata for the extension. This is important for the extension to be able to access the network and hardware devices. Paste in this json object:
```json
{
  "NetworkMode": "host",
  "HostConfig": {
    "Binds": [
      "/dev:/dev:rw"
    ],
    "Privileged": true,
    "NetworkMode": "host"
  }
}
```

Clicking "Create" should download the docker image (this can take a few minutes). The extension will then start up automatically and should run automatically when the pi reboots. When the extension is running, "ROS2" tab should appear in the left sidebar, and clicking it should open a terminal in which ros is running. The extension runs `ros2 launch pi_main pi_launch.py` on startup.

## Usage
The BlueOS web dashboard contains just about all the tools you need to monitor and control the Pi. The "Vehicle Setup" tab can be used to test the thrusters, assign output pins to motor numbers, and reverse thrusters as needed.


## Nodes

There are no nodes in this package.

## Launch files

* **pi_launch.py**: launch the manipulators, camera streaming, and pixhawk packages

