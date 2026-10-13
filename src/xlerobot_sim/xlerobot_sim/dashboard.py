"""Run the shared dashboard with the selected robot description and controls."""
import os
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
from .configuration import description_path


def main():
    configuration = sys.argv[1]
    runtime = Path('/workspace/xlerobot/.runtime')
    description = description_path(configuration, runtime)
    share = Path(get_package_share_directory('xlerobot_sim'))
    config = share / 'config' / ('dashboard_mod101.yaml' if configuration == 'mod101' else 'dashboard.yaml')
    os.execvp('ros2', ['ros2', 'run', 'robocore_viz', 'robocore_viz_node', '--ros-args',
        '-p', f'config_file:={config}', '-p', f'robot_description_file:={description}', *sys.argv[2:]])


if __name__ == '__main__':
    main()
