from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def get_config(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    los_rate_config = LaunchConfiguration('los_rate_config').perform(context)

    # Resolve paths (absolute or relative to package share)
    if os.path.isabs(config_file):
        vehicles_config_path = config_file
    else:
        package_share_dir = get_package_share_directory('gimbal_stabilizer')
        vehicles_config_path = os.path.join(package_share_dir, config_file)

    if os.path.isabs(los_rate_config):
        los_rate_config_path = los_rate_config
    else:
        package_share_dir = get_package_share_directory('gimbal_stabilizer')
        los_rate_config_path = os.path.join(package_share_dir, los_rate_config)

    if not os.path.exists(vehicles_config_path):
        raise FileNotFoundError(f"Vehicles config file not found: {vehicles_config_path}")
    if not os.path.exists(los_rate_config_path):
        raise FileNotFoundError(f"LOS rate config file not found: {los_rate_config_path}")

    with open(vehicles_config_path, 'r') as f:
        vehicles_config = yaml.safe_load(f)
    with open(los_rate_config_path, 'r') as f:
        los_rate_params = yaml.safe_load(f)

    return vehicles_config, los_rate_params


def launch_setup(context):
    vehicles_config, los_rate_params = get_config(context)
    gimbal_controller_mode = LaunchConfiguration('gimbal_controller_mode').perform(context)
    zoom_max = LaunchConfiguration('zoom_max').perform(context)
    nodes = []

    for vehicle in vehicles_config['vehicles']:
        namespace = vehicle['namespace']

        # Merge vehicle-specific params with LOS rate controller params
        node_params = los_rate_params['los_rate_controller'].copy()
        if 'los_rate' in vehicle:
            node_params.update(vehicle['los_rate'])
        # CLI argument overrides config file
        if gimbal_controller_mode:
            node_params['gimbal_controller_mode'] = gimbal_controller_mode
        if zoom_max:
            node_params['zoom_max'] = float(zoom_max)

        node = Node(
            package='gimbal_stabilizer',
            executable='los_rate_controller',
            name='los_rate_controller',
            namespace=namespace,
            parameters=[{
                'use_sim_time': True,
                'vehicle_name': namespace,
                'update_rate': vehicle.get('update_rate', 100.0),
                'model': vehicle.get('model', 'iris_gimbal3'),
                **node_params
            }]
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='config/vehicles.yaml',
            description='Path to the vehicle configuration file'
        ),
        DeclareLaunchArgument(
            'los_rate_config',
            default_value='config/los_rate_config_aggressive.yaml',
            description='Path to the LOS rate controller configuration file'
        ),
        DeclareLaunchArgument(
            'gimbal_controller_mode',
            default_value='',
            description='Gimbal controller mode: "analytical" or "jacobian" (overrides config file, empty = use config)'
        ),
        DeclareLaunchArgument(
            'zoom_max',
            default_value='',
            description='Maximum zoom level (overrides config file, empty = use config)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
