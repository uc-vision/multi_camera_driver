from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
  pkg_base = FindPackageShare("multi_camera_driver").find('multi_camera_driver')
  config_folder = os.path.join(pkg_base, 'config')

  default_device = 'cuda:0'
  device = LaunchConfiguration('device')
  device_launch_arg = DeclareLaunchArgument(
    'device',
    default_value=default_device,
    description='Device?? '
    )

  default_camera_set = os.path.join(config_folder, 'camera_sets', '.yaml')
  camera_set = LaunchConfiguration('camera_set')
  camera_set_launch_arg = DeclareLaunchArgument(
    'camera_set',
    default_value=default_camera_set,
    description='Set of cameras to be used'
    )

  default_camera_settings = os.path.join(config_folder, 'camera_settings', 'camera_12p.yaml')
  camera_settings = LaunchConfiguration('camera_settings')
  camera_settings_launch_arg = DeclareLaunchArgument(
    'camera_settings',
    default_value=default_camera_settings,
    description='Param file for the twist_to_motor node '
    )
  
  camera_array_node = Node(
    package='multi_camera_driver',
    executable='camera_array',
    parameters=[camera_set, camera_settings, {'device': device}]
    )

  return LaunchDescription([
    device_launch_arg,
    camera_set_launch_arg,
    camera_settings_launch_arg,
    camera_array_node
  ])