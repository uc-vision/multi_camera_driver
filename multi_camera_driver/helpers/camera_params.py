from enum import Enum
import rospy2 as rospy

from beartype.typing import Dict, Any
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor


class Transform(Enum):
  none = 0
  rotate_90 = 1
  rotate_180 = 2
  rotate_270 = 3
  transpose = 4
  flip_horiz = 5
  flip_vert = 6
  transverse = 7

  
class ToneMapper(Enum):
  linear = 1
  reinhard = 2

def declare_read_only_parameters():


  calibration_file_descriptor = ParameterDescriptor(
    name='calibration_file', 
    type=4,
    read_only=True,
    description='calibration_file'
  )
  rospy._node.declare_parameter('calibration_file', '', calibration_file_descriptor)

  camera_set_file_descriptor = ParameterDescriptor(
    name='camera_set_file', 
    type=4,
    read_only=True,
    description='camera_set_file'
  )
  rospy._node.declare_parameter('camera_set_file', '', camera_set_file_descriptor)

  settings_file_descriptor = ParameterDescriptor(
    name='settings_file', 
    type=4,
    read_only=True,
    description='settings_file'
  )
  rospy._node.declare_parameter('settings_file', '', settings_file_descriptor)

  settings_file_descriptor = ParameterDescriptor(
    name='reset_cycle', 
    type=1,
    read_only=True,
    description='reset_cycle'
  )
  rospy._node.declare_parameter('reset_cycle', True, settings_file_descriptor)
  

  tracking_frame_descriptor = ParameterDescriptor(
    name='tracking_frame', 
    type=4,
    read_only=True,
    description='tracking_frame'
  )
  rospy._node.declare_parameter('tracking_frame', 'camera_ref', tracking_frame_descriptor)


  rig_frame_descriptor = ParameterDescriptor(
    name='rig_frame', 
    type=4,
    read_only=True,
    description='rig_frame'
  )
  rospy._node.declare_parameter('rig_frame', 'camera_bar', rig_frame_descriptor)


  rig_frame_descriptor = ParameterDescriptor(
    name='timeout_msec', 
    type=2,
    read_only=True,
    description='timeout_msec'
  )
  rospy._node.declare_parameter('timeout_msec', 1000, rig_frame_descriptor)

  rig_frame_descriptor = ParameterDescriptor(
    name='sync_threshold_msec', 
    type=2,
    read_only=True,
    description='sync_threshold_msec'
  )
  rospy._node.declare_parameter('sync_threshold_msec', 10, rig_frame_descriptor)




def declare_camera_parameters(default_settings: Dict[str, Any]):

  default_exposure = default_settings.get('exposure', 4000)
  exposure_desc = ParameterDescriptor(
    name='exposure', 
    type=2, 
    description='Exposure', 
    integer_range=[IntegerRange(from_value=100, to_value=200000)]
  )
  rospy._node.declare_parameter('exposure', default_exposure, descriptor = exposure_desc)

  default_gain = default_settings.get('gain', 0.5)
  gain_desc = ParameterDescriptor(
    name='gain', 
    type=3, 
    description='Gain', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=27.0)]
  )
  rospy._node.declare_parameter('gain', default_gain, descriptor = gain_desc)

  default_balance = default_settings.get('balance_ratio', 0.0)
  balance_desc = ParameterDescriptor(
    name='balance_ratio', 
    type=3, 
    description='Balance ratio', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=4.0)]
  )
  rospy._node.declare_parameter('balance_ratio', default_balance, descriptor = balance_desc)

  default_max_framerate = default_settings.get('max_framerate', 14)
  max_framerate_desc = ParameterDescriptor(
    name='max_framerate', 
    type=3, 
    description='Max Framerate', 
    floating_point_range=[FloatingPointRange(from_value=1.6, to_value=23.3)]
  )
  rospy._node.declare_parameter('max_framerate', default_max_framerate, descriptor=max_framerate_desc)

  default_binning = default_settings.get('binning', 1)
  binning_desc = ParameterDescriptor(
    name='binning', 
    type=2, 
    description='Binning', 
    integer_range=[IntegerRange(from_value=1, to_value=2, step=1)],
    additional_constraints="""{ "1": "Full resolution", "2": "Half Resolution" }"""
  )
  rospy._node.declare_parameter('binning', default_binning, descriptor = binning_desc)

  default_width = default_settings.get('resize_width', 0)
  resize_width_desc = ParameterDescriptor(
    name='resize_width', 
    type=2, 
    description='Resize Width', 
    integer_range=[IntegerRange(from_value=0, to_value=4096)]
  )
  rospy._node.declare_parameter('resize_width', default_width, descriptor = resize_width_desc)

  default_jpeg_quality = default_settings.get('jpeg_quality', 95)
  jpeg_quality_desc = ParameterDescriptor(
    name='jpeg_quality', 
    type=2, 
    description='Jpeg quality', 
    integer_range=[IntegerRange(from_value=1, to_value=100)]
  )
  rospy._node.declare_parameter('jpeg_quality', default_jpeg_quality, descriptor = jpeg_quality_desc)

  default_tone_mapping = ToneMapper[default_settings.get('tone_mapping', 'reinhard')]
  tone_mapping_desc = ParameterDescriptor(
    name='tone_mapping', 
    type=2, 
    description='Tonemapping method',
    integer_range=[IntegerRange(from_value=1, to_value=2)],
    additional_constraints= str({ data.name:data.value for data in ToneMapper })
  )
  rospy._node.declare_parameter('tone_mapping', default_tone_mapping.value, descriptor = tone_mapping_desc)

  default_transform = Transform[default_settings.get('transform', 'none')]
  transform_desc = ParameterDescriptor(
    name='transform', 
    type=2, 
    description='Transform image',
    additional_constraints= str({ data.name:data.value for data in Transform })
  )
  rospy._node.declare_parameter('transform', default_transform.value, descriptor = transform_desc)

  default_tone_gamma = default_settings.get('tone_gamma', 1.0)
  tone_gamma_desc = ParameterDescriptor(
    name='tone_gamma', 
    type=3, 
    description='Tonemap Gamma', 
    floating_point_range=[FloatingPointRange(from_value=0.25, to_value=4.0)]
  )
  rospy._node.declare_parameter('tone_gamma', default_tone_gamma, descriptor = tone_gamma_desc)

  default_tone_intensity = default_settings.get('tone_intensity', 1.0)
  tone_intensity_desc = ParameterDescriptor(
    name='tone_intensity', 
    type=3, 
    description='Tonemap Intensity', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=4.0)]
  )
  rospy._node.declare_parameter('tone_intensity', default_tone_intensity, descriptor = tone_intensity_desc)

  default_color_adapt = default_settings.get('color_adapt', 0.0)
  color_adapt_desc = ParameterDescriptor(
    name='color_adapt', 
    type=3, 
    description='Tonemap Color Adapt', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]
  )
  rospy._node.declare_parameter('color_adapt', default_color_adapt, descriptor = color_adapt_desc)

  default_light_adapt = default_settings.get('light_adapt', 0.5)
  light_adapt_desc = ParameterDescriptor(
    name='light_adapt', 
    type=3, 
    description='Tonemap Light Adapt', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]
  )
  rospy._node.declare_parameter('light_adapt', default_light_adapt, descriptor = light_adapt_desc)

  default_moving_average = default_settings.get('moving_average', 0.05)
  moving_average_desc = ParameterDescriptor(
    name='moving_average', 
    type=3, 
    description='Moving average alpha', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]
  )
  rospy._node.declare_parameter('moving_average', default_moving_average, descriptor = moving_average_desc)

