import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

THIS_PKG = 'cable_pose_offload'

def generate_launch_description():
    pkg_share = get_package_share_directory(THIS_PKG)
    
    config_path = os.path.join(pkg_share, 'config', 'cables_params.yaml')
    model_absolute_path = os.path.join(pkg_share, 'models', 'cables_model.pt')

    with open(config_path, "r") as f:
        params = yaml.safe_load(f)

    nodes = []

    for node_name, params_per_node in params.items():
        node_params = params_per_node.get('ros__parameters', params_per_node)

        nodes.append(
            Node(
                package=THIS_PKG,
                executable='detector_node',
                name=f"cable_pose_{node_name}",
                parameters=[
                    node_params, 
                    {'model_path': model_absolute_path} 
                ],
                output='screen',
            )
        )

    return LaunchDescription(nodes)
