rosdep update --rosdistro=$ROS_DISTRO
rosdep install --from-paths src --ignore-src -r -y
# Stolen from colcon build command in VsCode
colcon build --symlink-install
source install/setup.bash