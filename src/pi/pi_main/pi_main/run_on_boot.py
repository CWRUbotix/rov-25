"""When run sets up environment for the robot to run on boot."""
import os
import sys
import subprocess
import pathlib

from ament_index_python.packages import get_package_share_directory


def main() -> None:
    """
    Set up Pi file environment.

    Copies udev rules from this package into udev folder.

    """
    pi_main_share = get_package_share_directory('pi_main')

    file_location = pathlib.Path(__file__).parent.resolve()
    udev_script = os.path.join(file_location, 'copy.py')

    cmd = ['/usr/bin/sudo', '/usr/bin/python3', udev_script, pi_main_share]

    try:
        process = subprocess.run(cmd, capture_output=True, check=True)
    # Logs Error
    except subprocess.CalledProcessError as error:
        print(error.stderr)
        sys.exit(1)

    # Success Message
    print(process.stdout.decode())

    # TODO would be nice to copy service file.
    # Also this file could get refactored.
