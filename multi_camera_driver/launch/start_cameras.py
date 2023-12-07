from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml

def launch_setup(context):
  device = LaunchConfiguration('device').perform(context)
  camera_set = LaunchConfiguration('camera_set').perform(context)
  camera_settings = LaunchConfiguration('camera_settings').perform(context)

  # Pass path to start_cameras and declare all variables that way
  camera_array_node = Node(
    package='multi_camera_driver',
    executable='start_cameras',
    parameters=[{
      'device': device, 
      'camera_set_file': camera_set, 
      'settings_file': camera_settings
      }],
    )
  return [camera_array_node]


def generate_launch_description():
  pkg_base = FindPackageShare("multi_camera_driver").find('multi_camera_driver')
  config_folder = os.path.join(pkg_base, 'config')
  
  default_device = 'cuda:0'
  device_launch_arg = DeclareLaunchArgument(
    'device',
    default_value=default_device,
    description='Torch device for GPU image processing.')

  default_camera_set = os.path.join(config_folder, 'camera_sets', 'eth_bar.yaml')
  camera_set_launch_arg = DeclareLaunchArgument(
    'camera_set',
    default_value=default_camera_set,
    description='Set of cameras to be used')

  default_camera_settings = os.path.join(config_folder, 'camera_settings', 'camera_12p_eth_sync.yaml')
  camera_settings_launch_arg = DeclareLaunchArgument(
    'camera_settings',
    default_value=default_camera_settings,
    description='Param file for the twist_to_motor node ')
  return LaunchDescription([camera_settings_launch_arg, 
                            camera_set_launch_arg, 
                            device_launch_arg] 
                            + [OpaqueFunction(function=launch_setup)])
