# MATE ROV 2024-25

[![Continuous Integration](https://github.com/CWRUbotix/rov-25/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/CWRUbotix/rov-25/actions/workflows/industrial_ci_action.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/CWRUbotix/rov-25/main.svg)](https://results.pre-commit.ci/latest/github/CWRUbotix/rov-25/main)
[![Apache License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Ruff](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json)](https://github.com/astral-sh/ruff)

# Table of Contents

1. [Setup](#setup)
    1. [Linux](#linux)
        1. [Docker](#docker)
        2. [Bare Metal](#bare-metal)
    2. [Windows](#windows)
        1. [Docker](#docker-1)
        2. [WSL](#wsl)
    3. [macOS](#macos)
        1. [Docker](#docker-2)
2. [Test Your Environment](#test-your-environment)
3. [Code Building](#building-with-colcon)
4. [Directory Structure](#directory-structure)
5. [Documentation Structure](#documentation-structure)

## Setup

If you have GitHub Desktop, click the green `Code` button above, then `Open with GitHub Desktop`.

If you have the Git CLI, start by opening up a terminal, navigating to where you want the code to be saved, and entering the following command:

```bash
git clone --recurse-submodules git@github.com:CWRUbotix/rov-25.git
```

If you've never contributed to a git repository before, you might receive an error message saying you don't have access. In that case visit [this tutorial](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/about-ssh) to set up SSH for local GitHub access.

After cloning the code, we need to set up our IDE: VSCode. If you already have it, great. Otherwise follow [this](https://code.visualstudio.com/download) tutorial. We recommend installing the mypy and autoDocstring VSCode extensions. Our autoDocstring settings are `Docstring Format: Numpy` and `Start On New Line: Start docstring on new line`.

Now select the guide below that fits your operating system. Some operating systems have multiple options. Here are our recommended installation options:
 - **Linux**: If you have the latest Ubuntu LTS, use the Bare Metal installation. Otherwise you'll need to do the Docker install.
 - **Windows**: You should create a dual boot, then follow the Linux instructions for your new Ubuntu OS. Otherwise, we recommend using WSL, although Docker has similar capabilities.
 - **MacOS**: We recommend using Docker. UTM should theoretically have more capabilities than Docker, but it has a long setup time and we're not sure it works.

Once you're finished setting up, go to the [Test Your Environment](#test-your-environment) section to confirm everything's working.

### Linux

#### Bare Metal (Ubuntu only)

To run the install script, open the repository with VSCode. Then use `F1` or `ctrl+shift+p` to open the command bar and select `Tasks: Run Task`. Then from the Task selection choose `[SETUP] Surface Environment`. This will install ROS and all our dependencies.

#### Docker (all distros, probably)

Start by installing docker engine from [here](https://docs.docker.com/engine/install/ubuntu/).

Set your permissions based on [this](https://docs.docker.com/engine/install/linux-postinstall/) guide.

Restart.

Then install the Dev Containers VSCode extension.

To open the container, open the repository with VSCode. Then use `F1` or `ctrl+shift+p` to open the command bar and select `Tasks: Run Task`. Then from the Task selection choose `Docker Rebuild`. This will build and run the docker container. Make sure to choose `ROV Linux` for which type to run (you might have to wait a while for things to download before this option is presented at the top of your screen).

Now you should be in your Docker container in VSCode.

To reopen the container after you close it, go to `File` > `Open Recent` > `/stuff/rov-25 [Dev Container]`

For gui apps run `xhost + local:docker` before launching docker or add to `.bashrc`.

### Windows

#### Dual Boot

Creating a dual boot allows you to install two different operating systems to your device. You can then switch between them when your computer boots. Once you have a dual boot set up, this is by far the easiest and most error-free option. You'll also find that many CS courses are easier once you have easy access to a full Linux OS.

[This is a guide](https://www.tomshardware.com/how-to/dual-boot-linux-and-windows-11#how-to-install-linux-for-dual-boot-3) for creating a dual boot. Note that there a bootable USB drives available in the bay for Ubuntu 24.04, so you can skip to the "How to Install Linux for Dual Boot" section.

You might need to [disable secure boot](https://learn.microsoft.com/en-us/windows-hardware/manufacture/desktop/disabling-secure-boot?view=windows-11), but try without doing that first. If you get secure boot errors later in the installation, that might be because you chose to [install third party drivers](https://askubuntu.com/questions/1513173/do-i-need-to-disable-secure-boot-to-install-ubuntu-24-04).

Once you have Ubuntu set up, follow the [Bare Metal (Ubuntu only)](#bare-metal-ubuntu-only) instructions in that OS.

#### WSL

Follow [this](https://learn.microsoft.com/en-us/windows/wsl/install) guide to install WSL.

After WSL has been installed follow [this](https://code.visualstudio.com/docs/remote/wsl) guide to get VSCode and WSL to properly communicate and navigate to the rov-25 folder.

Then run the install script.

To run the install script, open the repository with VSCode. Then use `F1` or `ctrl+shift+p` to open the command bar and select `Tasks: Run Task`. Then from the Task selection choose `[SETUP] Surface Environment`. This will install ROS and all our dependencies.

#### Docker

Start by installing docker from [here](https://www.docker.com/get-started/).

Then install the Dev Containers VSCode extension.

To open the container, open the repository with VSCode. Then use `F1` or `ctrl+shift+p` to open the command bar and select `Tasks: Run Task`. Then from the Task selection choose `Docker Rebuild`. This will build and run the docker container. Make sure to choose `ROV Windows` for which type to run (you might have to wait a while for things to download before this option is presented at the top of your screen).

Now you should be in your Docker container in VSCode.

To reopen the container after you close it, go to `File` > `Open Recent` > `/stuff/rov-25 [Dev Container]`

To add your Git SSH keys into the container follow [this](https://code.visualstudio.com/remote/advancedcontainers/sharing-git-credentials) guide.

If this doesn't work try running `ssh -v git@github.com`

If get_agent_identities from the prior command is empty you will likely need to download a newer version of OpenSSH from [here](https://github.com/PowerShell/Win32-OpenSSH/releases/tag/v9.4.0.0p1-Beta). Download the .msi file. Once download double click to update it. You might need to do the previous step again.

To get GUI support inside Docker, download [this](https://sourceforge.net/projects/vcxsrv/files/latest/download).

Then run XLaunch from the Windows Start Menu. Make the settings look like this:

![Screenshot of the settings for Xlaunch](/doc/images/VcXsrv.png)

Then open up Command Prompt and type `ipconfig`.

Then in the terminal of the docker container use `export DISPLAY={IPv4 of WSL}:0.0` where the IPV4 is from the `ipconfig` command.

### macOS

#### Docker

First, install Docker from [here](https://www.docker.com/get-started/).

Then install the `Dev Containers` extension inside VSCode.

To get GUI support inside Docker, first install [homebrew](https://brew.sh/).

Then, from a terminal, run the following:

```bash
brew install socat
brew install xquartz
```

Next, configure XQuartz with

```bash
open -a Xquartz
```

Go to the settings menu and make the `Security` section look like this:

![image](/doc/images/macos_xserver_settings.jpg)

You can close out of the XQuartz terminal now, but you should always see the XQuartz icon in your dock whenever you want a GUI to display from inside the Docker container.

To open the container, open our repository with VSCode. Then use `F1` or `command` `shift` `p` to open the command bar and select `Tasks: Run Task`. Then from the Task selection choose `Docker Rebuild`. This will build and run the docker container. Make sure to choose `ROV macOS` for which type to run (you might have to wait a while for things to download before this option is presented at the top of your screen).

Now you should be in your Docker container in VSCode.

To reopen the container after you close it, go to `File` > `Open Recent` > `/stuff/rov-25 [Dev Container]`.

> Note for folks who eventually try running the simulation: XQuartz might not be able to support OpenGL applications like our simulation. [Apparently there's a flag](https://unix.stackexchange.com/questions/429760/opengl-rendering-with-x11-forwarding) to enable OpenGL rendering as late at XQuartz 2.7.11, but that version might be too old to run on your Mac.

<!-- https://gist.github.com/cschiewek/246a244ba23da8b9f0e7b11a68bf3285 -->

#### UTM

> BEWARE! We haven't fully tested this option.

[UTM](https://docs.getutm.app/installation/macos/) is a virtualization software. It allows you to create "virtual machines" (sometimes called "guests") which can run other operating systems. Visit [UTM](https://docs.getutm.app/installation/macos/) and hit "Download from GitHub" (downloading from the App Store costs money).

As mentioned in this [StackExchange](https://apple.stackexchange.com/questions/434799/utm-ubuntu-linux-installation-says-network-is-unreachable?rq=1), if you have issues getting a network connection with either of these options, try changing the network mode of your virtual machine in its settings menu (the icon with sliders on the top right when the virtual machine is selected).

If you have Apple silicon (an M1, M2, etc.):
 - Download a disk image of [Ubuntu 24.04 Server for ARM64](https://ubuntu.com/download/server/arm).
 - [This Youtube video](https://www.youtube.com/watch?v=JrNS3brSnmA) provides a good tutorial on what to do next.
 - Once the installation is complete, follow the instructions in this README for [Bare Metal (Ubuntu only)](#bare-metal-ubuntu-only).
 - You'll probably run into issues with our packages being built for AMD64 instead of ARM64. This is an uninvestigated problem, but emulating AMD64 on ARM Macs is too slow to be feasible.

If you have a different (i.e. Intel) CPU:
 - Download a disk image of [Ubuntu 24.04 Desktop for AMD64](https://ubuntu.com/download/desktop).
 - Follow these instructions stolen from [UTM's guide](https://docs.getutm.app/guides/ubuntu/):
   - Open UTM and click the `+` button to open the VM creation wizard.
   - Select `Virtualize`.
   - Select `Linux`.
   - Click `Browse` and select the Ubuntu  ISO downloaded from the link above. Press `Continue` to continue.
   - Pick the amount of RAM and CPU cores you wish to give access to the VM. Press `Continue` to continue.
   - Specify the maximum amount of drive space to allocate. Press `Continue` to continue.
   - If you have a directory you want to mount in the VM (so you can share files between your MacOS filesystem and the virtual machine's filesystem), you can select it here. The shared directory will be available after installing SPICE tools (see [here](https://docs.getutm.app/guest-support/linux/) to install SPICE Agent and SPICE WebDAV). If you don't understand what's going on here, don't change anything. Press `Continue` to continue.
   - Press `Save` to create the VM and press the play button to start the VM.
   - Go through the Ubuntu installer. If the reboot fails, you can manually quit the VM, unmount the installer ISO, and start the VM again to boot into your new installation.
 - Once the installation is complete, follow the instructions in this README for [Bare Metal (Ubuntu only)](#bare-metal-ubuntu-only).



## Test Your Environment

After setting up your environment, open a terminal (make sure it's in VSCode if you're using Docker) and run an example publisher node:

```bash
ros2 run demo_nodes_cpp talker
```

Open a second terminal and run an example subscriber node:

```bash
ros2 run demo_nodes_py listener
```

If that works, try launching our GUI:

```bash
ros2 launch surface_main surface_all_nodes_launch.py
```

You might have to build (see the next section for how to do that) and then source your ROS workspace first if ROS can't find `surface_main`.

To source your workspace, run:

```bash
. install/setup.sh
```


## Building with Colcon

Now, anytime you want to build, perform one of these options in VSCode:
 - Press `ctrl` `shift` `b`
 - Press `F1` or `ctrl` `shift` `p`, choose `Tasks: Run Task`, then choose `[ROS] 🏃‍ Build Workspace`

The magic of symlinks should mean you won't need to build again for most things, but you'll need to build every time you change a package's `setup.py` or anything in the `rov_msgs` package.

## Unit tests

To test, perform one of these options in VSCode:
 - Press `F1` or `ctrl` `shift` `p` and choose `Tasks: Run Test Task`
 - Run `colcon test --event-handlers=console_direct+`

The runs the tests and pipes the output to the terminal. To test `pi_main` make sure to type your password into the terminal after doing either option..

Installing the mypy extension should help enforce our linters.

<!-- ### Automatic building for non-VSCode heathens

Run this command from your workspace folder

The magic of symlink should mean you won't need to build again for most
things, but if you're working on package metadata (e.g. `package.xml`) or
rov_msgs, you'll need to run this every time you change something:

```bash
. src/.vscode/easy_build.sh
``` -->

## Upgrading Environment

If you are upgrading to a newer ROS version make sure to remove `source /opt/ros/$PREVIOUS_ROS_DISTRO/setup.bash`. If you are using Docker you can simply build the new container and delete the old one.

## Directory Structure

All packages to be installed on the surface computer live in the `surface` directory.

All packages to be installed on the pi compute module live in the `pi` directory.

All packages to be installed on the float live in the `float` directory.

## Namespaces

All nodes running on the pi will be in the pi namespace.

All nodes running on the surface will be in the surface namespace.

Any topics or services communicating across will be renamed first into the tether namespace.

![Picture of rqt with namespaces](/doc/images/namespaces.png)

## Documentation Structure

Documentation will take place at 3 levels:

- High Level - Overarching Design Document outlining our general structure and what goes where.
- Device Level - Following the markdown tempate in `doc` directory.
- Inline Level - Using reST / Numpy Standard. To autogenerate in VSCode we use autoDocstring extension with the setting set to Numpy and auto docstring on new line. Below is an example of an inline function docstring.

```python
def __init__(self, srv_type: SrvType, topic: str, signal: pyqtBoundSignal,
                 timeout: float = 1.0, expected_namespace: str = '/surface/gui'):
        """
        _summary_

        Parameters
        ----------
        srv_type : SrvType
            _description_
        topic : str
            _description_
        signal : pyqtBoundSignal
            _description_
        timeout : float, optional
            _description_, by default 1.0
        expected_namespace : str, optional
            _description_, by default '/surface/gui'
        """
```
