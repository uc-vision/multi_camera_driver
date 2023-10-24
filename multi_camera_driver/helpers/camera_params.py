import rospy2 as rospy

from beartype.typing import Dict, Any
from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor


TRANSFORM = [
  'none',
  'rotate_90',
  'rotate_180',
  'rotate_270',
  'transpose',
  'flip_horiz',
  'flip_vert',
  'transverse'
  ]

TONE_MAPPING = [
  None,
  'linear',
  'reinhard'
]

def declare_ros2_parameters(default_settings: Dict[str, Any]):

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
    additional_constraints="""
    Mapping: 
      1 = Full resolution
      2 = Half Resolution
    """
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

  default_tone_mapping = default_settings.get('tone_mapping', 'linear')
  default_tone_mapping_value = TONE_MAPPING.index(default_tone_mapping)
  tone_mapping_desc = ParameterDescriptor(
    name='tone_mapping', 
    type=2, 
    description='Tonemapping method',
    integer_range=[IntegerRange(from_value=1, to_value=2)],
    additional_constraints='Mapping: 1 = Linear, 2 = Reinhard'
  )
  rospy._node.declare_parameter('tone_mapping', default_tone_mapping_value, descriptor = tone_mapping_desc)

  default_transform = default_settings.get('transform', 'none')
  default_transform_value = TRANSFORM.index(default_transform)
  transform_desc = ParameterDescriptor(
    name='transform', 
    type=2, 
    description='Transform image',
    additional_constraints="""
    Mapping:
      0 = None
      1 = Rotate 90 degrees CW
      2 = Rotate 90 degrees CCW
      3 = Rotate 180 degrees CW
      4 = Transposed
      5 = Flip horizontally
      6 = Flip vertically
      7 = Flip horizontally and vertically
      """
  )
  rospy._node.declare_parameter('transform', default_transform_value, descriptor = transform_desc)

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

