"""Copies udev rules from separate process to ensure ideal protections of sudo."""

import shutil
import sys
from pathlib import Path

if __name__ == '__main__':
    SHARE_DIR = sys.argv[1]

    udev_src_dir = Path(SHARE_DIR) / 'udev_rules'
    udev_dst_dir = Path('/etc') / 'udev' / 'rules.d'

    shutil.copytree(udev_src_dir, udev_dst_dir, dirs_exist_ok=True)

    service_src = Path(SHARE_DIR) / 'services' / 'pi_main.service'

    service_dst_folder = Path('/etc') / 'systemd' / 'system'
    service_dst = service_dst_folder / 'pi_main.service'

    shutil.copy(service_src, service_dst)
    print('Copying udev rules and services')
