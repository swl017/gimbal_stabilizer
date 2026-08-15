"""Launch `fixed_mount_publisher` per vehicle — the fixed-camera (no gimbal) session.

MAS ticket 055.  This is the drop-in replacement for
`multi_agent_los_rate_aggressive.launch.py` in the fixed-camera Isaac session: same
`config/vehicles.yaml` fan-out, same namespaces, but each vehicle gets a constant
mount pose instead of a LOS-rate controller.

The two must not both run on a namespace — they both publish `isaac_joint_commands`.

`namespaces:=px4_1,px4_2` restricts which vehicles get the fixed mount, so a MIXED
session (fixed-camera interceptor + gimballed observers) can be assembled by launching
this for one subset and `multi_agent_los_rate*.launch.py` for the complement.  Empty
(the default) means every vehicle in the config file.

Per-vehicle overrides go in `vehicles.yaml` under a `fixed_mount:` key, mirroring how
the LOS-rate launch reads `los_rate:`:

    vehicles:
      - namespace: "px4_1"
        model: "iris_gimbal3"
        fixed_mount:
          mount_pitch_up_deg: 30.8
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve(path: str) -> str:
    if os.path.isabs(path):
        return path
    return os.path.join(get_package_share_directory('gimbal_stabilizer'), path)


def launch_setup(context):
    config_path = _resolve(LaunchConfiguration('config_file').perform(context))
    if not os.path.exists(config_path):
        raise FileNotFoundError(f'Vehicles config file not found: {config_path}')
    with open(config_path, 'r') as f:
        vehicles_config = yaml.safe_load(f)

    selected = [ns.strip() for ns in
                LaunchConfiguration('namespaces').perform(context).split(',')
                if ns.strip()]
    mount_pitch_up_deg = LaunchConfiguration('mount_pitch_up_deg').perform(context)
    publish_rate_hz = LaunchConfiguration('publish_rate_hz').perform(context)

    nodes = []
    for vehicle in vehicles_config['vehicles']:
        namespace = vehicle['namespace']
        if selected and namespace not in selected:
            continue

        node_params = dict(vehicle.get('fixed_mount', {}))
        # CLI argument overrides the config file, as in the LOS-rate launch.
        if mount_pitch_up_deg:
            node_params['mount_pitch_up_deg'] = float(mount_pitch_up_deg)
        if publish_rate_hz:
            node_params['publish_rate_hz'] = float(publish_rate_hz)

        nodes.append(Node(
            package='gimbal_stabilizer',
            executable='fixed_mount_publisher',
            name='fixed_mount_publisher',
            namespace=namespace,
            parameters=[{
                'use_sim_time': True,
                'model': vehicle.get('model', 'iris_gimbal3'),
                **node_params,
            }],
        ))

    if not nodes:
        raise RuntimeError(
            f'No vehicles selected: namespaces={selected or "<all>"} matched none of '
            f'{[v["namespace"] for v in vehicles_config["vehicles"]]} in {config_path}')

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='config/vehicles.yaml',
            description='Path to the vehicle configuration file'
        ),
        DeclareLaunchArgument(
            'namespaces',
            default_value='',
            description='Comma-separated namespaces to give a fixed mount '
                        '(empty = every vehicle in the config file)'
        ),
        DeclareLaunchArgument(
            'mount_pitch_up_deg',
            default_value='',
            description='Mount pitch, positive UP [deg] (overrides the config file; '
                        'empty = node default 30.8, the flown interceptor)'
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='',
            description='Publish rate (overrides the config file, empty = node default)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
