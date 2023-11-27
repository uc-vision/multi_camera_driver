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

  camera_set_file_descriptor = ParameterDescriptor(
    name='camera_set_file', 
    type=4,
    read_only=True,
    description='camera_set_file'
  )

  settings_file_descriptor = ParameterDescriptor(
    name='settings_file', 
    type=4,
    read_only=True,
    description='settings_file'
  )

  reset_cycle_descriptor = ParameterDescriptor(
    name='reset_cycle', 
    type=1,
    read_only=True,
    description='reset_cycle'
  )

  tracking_frame_descriptor = ParameterDescriptor(
    name='tracking_frame', 
    type=4,
    read_only=True,
    description='tracking_frame'
  )

  rig_frame_descriptor = ParameterDescriptor(
    name='rig_frame', 
    type=4,
    read_only=True,
    description='rig_frame'
  )

  timeout_msec_descriptor = ParameterDescriptor(
    name='timeout_msec', 
    type=2,
    read_only=True,
    description='timeout_msec'
  )

  sync_threshold_msec_descriptor = ParameterDescriptor(
    name='sync_threshold_msec', 
    type=2,
    read_only=True,
    description='sync_threshold_msec'
  )

  rospy._node.declare_parameters(
    namespace='',
    parameters=[
      ('calibration_file', '', calibration_file_descriptor),
      ('camera_set_file', '', camera_set_file_descriptor),
      ('settings_file', '', settings_file_descriptor),
      ('reset_cycle', True, reset_cycle_descriptor),
      ('tracking_frame', 'camera_ref', tracking_frame_descriptor),
      ('rig_frame', 'camera_bar', rig_frame_descriptor),
      ('timeout_msec', 1000, timeout_msec_descriptor),
      ('sync_threshold_msec', 10, sync_threshold_msec_descriptor),
    ]
  )




def declare_camera_parameters(default_settings: Dict[str, Any]):

  default_exposure = default_settings.get('exposure', 4000)
  exposure_desc = ParameterDescriptor(
    name='exposure', 
    type=2, 
    description='Exposure', 
    integer_range=[IntegerRange(from_value=100, to_value=200000)]
  )

  default_gain = default_settings.get('gain', 0.5)
  gain_desc = ParameterDescriptor(
    name='gain', 
    type=3, 
    description='Gain', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=27.0)]
  )

  default_balance = default_settings.get('balance_ratio', 0.0)
  balance_desc = ParameterDescriptor(
    name='balance_ratio', 
    type=3, 
    description='Balance ratio', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=4.0)]
  )

  default_max_framerate = default_settings.get('max_framerate', 14)
  max_framerate_desc = ParameterDescriptor(
    name='max_framerate', 
    type=3, 
    description='Max Framerate', 
    floating_point_range=[FloatingPointRange(from_value=1.6, to_value=23.3)]
  )

  default_binning = default_settings.get('binning', 1)
  binning_desc = ParameterDescriptor(
    name='binning', 
    type=2, 
    description='Binning', 
    integer_range=[IntegerRange(from_value=1, to_value=2, step=1)],
    additional_constraints="""{ "1": "Full resolution", "2": "Half Resolution" }"""
  )

  default_width = default_settings.get('resize_width', 0)
  resize_width_desc = ParameterDescriptor(
    name='resize_width', 
    type=2, 
    description='Resize Width', 
    integer_range=[IntegerRange(from_value=0, to_value=4096)]
  )

  default_jpeg_quality = default_settings.get('jpeg_quality', 95)
  jpeg_quality_desc = ParameterDescriptor(
    name='jpeg_quality', 
    type=2, 
    description='Jpeg quality', 
    integer_range=[IntegerRange(from_value=1, to_value=100)]
  )

  default_tone_mapping = ToneMapper[default_settings.get('tone_mapping', 'reinhard')]
  tone_mapping_desc = ParameterDescriptor(
    name='tone_mapping', 
    type=2, 
    description='Tonemapping method',
    integer_range=[IntegerRange(from_value=1, to_value=2)],
    additional_constraints= str({ data.name:data.value for data in ToneMapper })
  )

  default_transform = Transform[default_settings.get('transform', 'none')]
  transform_desc = ParameterDescriptor(
    name='transform', 
    type=2, 
    description='Transform image',
    additional_constraints= str({ data.name:data.value for data in Transform })
  )

  default_tone_gamma = default_settings.get('tone_gamma', 1.0)
  tone_gamma_desc = ParameterDescriptor(
    name='tone_gamma', 
    type=3, 
    description='Tonemap Gamma', 
    floating_point_range=[FloatingPointRange(from_value=0.25, to_value=4.0)]
  )

  default_tone_intensity = default_settings.get('tone_intensity', 1.0)
  tone_intensity_desc = ParameterDescriptor(
    name='tone_intensity', 
    type=3, 
    description='Tonemap Intensity', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=4.0)]
  )

  default_color_adapt = default_settings.get('color_adapt', 0.0)
  color_adapt_desc = ParameterDescriptor(
    name='color_adapt', 
    type=3, 
    description='Tonemap Color Adapt', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]
  )

  default_light_adapt = default_settings.get('light_adapt', 0.5)
  light_adapt_desc = ParameterDescriptor(
    name='light_adapt', 
    type=3, 
    description='Tonemap Light Adapt', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]
  )

  default_moving_average = default_settings.get('moving_average', 0.05)
  moving_average_desc = ParameterDescriptor(
    name='moving_average', 
    type=3, 
    description='Moving average alpha', 
    floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]
  )
  rospy._node.declare_parameters(
    namespace = '',
    parameters=[
      ('exposure', default_exposure, exposure_desc),
      ('gain', default_gain, gain_desc),
      ('balance_ratio', default_balance, balance_desc),
      ('max_framerate', default_max_framerate, max_framerate_desc),
      ('binning', default_binning, binning_desc),
      ('resize_width', default_width, resize_width_desc),
      ('jpeg_quality', default_jpeg_quality, jpeg_quality_desc),
      ('tone_mapping', default_tone_mapping.value, tone_mapping_desc),
      ('transform', default_transform.value, transform_desc),
      ('tone_gamma', default_tone_gamma, tone_gamma_desc),
      ('tone_intensity', default_tone_intensity, tone_intensity_desc),
      ('color_adapt', default_color_adapt, color_adapt_desc),
      ('light_adapt', default_light_adapt, light_adapt_desc),
      ('moving_average', default_moving_average, moving_average_desc)
    ]
  )
  

