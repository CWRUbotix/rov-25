FROM osrf/ros:jazzy-desktop

ENV ROS_DISTRO jazzy
ENV USER_NAME root

RUN apt-get update -y \
  && apt-get install --no-install-recommends -y \
  # Install Video for Linux
  v4l-utils -y \
  # Install lsusb
  usbutils \
  # Install nano
  nano \
  # Install ssh
  ssh \
  # Install pip
  python3-pip \
  # Install geographiclib dependencies for mavros.
  geographiclib-tools \
  && apt-get upgrade -y \
  # Clean for better performance
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

# Done here because file needs sudo perms
# Switch to bash so the process subsition works. aka <()
# Runs MAVROS helper install script
SHELL ["/bin/bash", "-c"]
RUN . <(wget -qO- https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh)

WORKDIR /home/
# Setup Ardusub
COPY src/surface/rov_gazebo/scripts/ardusub.sh .
RUN ./ardusub.sh \
 && rm ardusub.sh

# Setup Ardupilot Gazebo
COPY src/surface/rov_gazebo/scripts/ardupilot_gazebo.sh .
RUN ./ardupilot_gazebo.sh \
    && rm ardupilot_gazebo.sh

WORKDIR /home/${USER_NAME}/rov-25

COPY . .

RUN git submodule init && git submodule update

# TODO: can probably remove the arg passing when iron support is dropped
RUN .vscode/install_dependencies.sh

# TODO: for future nerd to do this via ENTRYPOINT which be better but, I could not get ENTRYPOINT to play with VsCODE.
RUN .vscode/rov_setup.sh

# Setup Models and Gazebo Environment
RUN ./src/surface/rov_gazebo/scripts/add_models_and_worlds.sh

# Do full build as last step
RUN source /opt/ros/${ROS_DISTRO}/setup.sh \
  && colcon build --symlink-install
