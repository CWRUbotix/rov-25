# Done to suppress setup.py install deprecated warnings
# Can be removed once ROS redoes their python build system
PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::setuptools.command.develop,ignore:::pkg_resources; export PYTHONWARNINGS

# Stolen from colcon build command in VsCode
colcon build --symlink-install
source install/setup.bash