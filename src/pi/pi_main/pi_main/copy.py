"""Copies udev rules from separate process to ensure ideal protections of sudo."""
import os
import shutil
import sys

if __name__ == '__main__':
    SHARE_DIR = sys.argv[1]

    udev_src_dir = os.path.join(SHARE_DIR, 'udev_rules')
    udev_dst_dir = os.path.join('/etc', 'udev', 'rules.d')

    shutil.copytree(udev_src_dir, udev_dst_dir, dirs_exist_ok=True)

    service_src = os.path.join(SHARE_DIR, 'services', 'pi_main.service')

    service_dst_folder = os.path.join('/etc', 'systemd', 'system')
    service_dst = os.path.join(service_dst_folder, 'pi_main.service')

    shutil.copy(service_src, service_dst)
    print("Copying udev rules and services")
