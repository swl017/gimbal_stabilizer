from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def get_config(context):
    config_file = LaunchConfiguration('config_file').perform(context)
    gimbal_config = LaunchConfiguration('gimbal_config').perform(context)
    
    # If absolute paths, use them directly
    if os.path.isabs(config_file):
        vehicles_config_path = config_file
    else:
        # If relative paths, look in the package's share directory
        package_share_dir = get_package_share_directory('gimbal_stabilizer')
        vehicles_config_path = os.path.join(package_share_dir, config_file)
        
    if os.path.isabs(gimbal_config):
        gimbal_config_path = gimbal_config
    else:
        package_share_dir = get_package_share_directory('gimbal_stabilizer')
        gimbal_config_path = os.path.join(package_share_dir, gimbal_config)
    
    if not os.path.exists(vehicles_config_path):
        raise FileNotFoundError(f"Vehicles config file not found: {vehicles_config_path}")
    if not os.path.exists(gimbal_config_path):
        raise FileNotFoundError(f"Gimbal config file not found: {gimbal_config_path}")
        
    with open(vehicles_config_path, 'r') as f:
        vehicles_config = yaml.safe_load(f)
    with open(gimbal_config_path, 'r') as f:
        gimbal_params = yaml.safe_load(f)
        
    return vehicles_config, gimbal_params

def launch_setup(context):
    vehicles_config, gimbal_params = get_config(context)
    nodes = []
    
    for vehicle in vehicles_config['vehicles']:
        namespace = vehicle['namespace']
        
        # Merge vehicle-specific params with gimbal params
        node_params = gimbal_params['gimbal_stabilizer'].copy()
        if 'gimbal' in vehicle:
            node_params.update(vehicle['gimbal'])
        
        node = Node(
            package='gimbal_stabilizer',
            executable='gimbal_stabilizer',
            name='gimbal_stabilizer',
            namespace=namespace,
            # remappings=[
            #     ('isaac_joint_commands', f'{namespace}/isaac_joint_commands'),
            #     ('isaac_joint_states', f'{namespace}/isaac_joint_states'),
            #     ('state/pose', f'{namespace}/state/pose'),
            # ],
            parameters=[{
                'vehicle_name': namespace,
                'update_rate': vehicle.get('update_rate', 100.0),
                **node_params
            }]
        )
        nodes.append(node)
    
    return nodes

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('gimbal_stabilizer')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='config/vehicles.yaml',
            description='Path to the vehicle configuration file'
        ),
        DeclareLaunchArgument(
            'gimbal_config',
            default_value='config/gimbal_config.yaml',
            description='Path to the gimbal configuration file'
        ),
        OpaqueFunction(function=launch_setup)
    ])