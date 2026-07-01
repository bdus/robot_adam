#!/usr/bin/env python3
"""
Cartographer map archiving script.

Usage:
  ros2 run adam_slam archive_map.py                    # archive to runtime cache
  ros2 run adam_slam archive_map.py --commit            # also sync to source tree
"""
import os
import sys
import shutil
import subprocess
from datetime import datetime

CACHE_DIR = os.path.expanduser('~/.ros/adam_assets/maps_2d')
CART_SAVE_SERVICE = '/write_state'  # Fixed service name

def main():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    tmp_dir = f'/tmp/adam_maps/{timestamp}'
    os.makedirs(tmp_dir, exist_ok=True)

    pbstream_path = f'{tmp_dir}/map_{timestamp}.pbstream'

    # Call Cartographer /write_state service
    cmd = [
        'ros2', 'service', 'call', CART_SAVE_SERVICE,
        'cartographer_ros_msgs/srv/WriteState',
        f'{{filename: "{pbstream_path}"}}',
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

    if result.returncode != 0:
        print(f'ERROR: Failed to write state: {result.stderr}')
        sys.exit(1)

    # Copy to runtime cache
    os.makedirs(CACHE_DIR, exist_ok=True)
    shutil.copy2(pbstream_path, f'{CACHE_DIR}/map_{timestamp}.pbstream')
    print(f'Archived {pbstream_path} → {CACHE_DIR}/')

    # --commit: also sync to source tree
    if '--commit' in sys.argv:
        src_dir = os.path.expanduser('src/navigation_ai/adam_assets/share/maps_2d')
        if os.path.exists(src_dir):
            shutil.copy2(pbstream_path, f'{src_dir}/map_{timestamp}.pbstream')
            subprocess.run(['colcon', 'build', '--packages-select', 'adam_assets'],
                           timeout=120)
            print(f'Committed to source tree: {src_dir}/')

if __name__ == '__main__':
    main()