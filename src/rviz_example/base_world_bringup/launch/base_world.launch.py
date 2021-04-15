import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('base_world_bringup').find('base_world_bringup')
    urdf_file = os.path.join(pkg_share, 'urdf', 'basic_world.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}
    return LaunchDescription([
        Node(package='laser_scan_publisher', executable='laser_scan_pub', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[rsp_params]),
        Node(package='dynamic_joint_states', executable='dynamic_joint_pub', output='screen'),
    ])
