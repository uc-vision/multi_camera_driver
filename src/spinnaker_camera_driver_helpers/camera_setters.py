from . import spinnaker_helpers
import PySpin

def set_exposure(camera : PySpin.Camera, exposure_time, info):
    """Set the cameras exposure time the the given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if exposure_time > 0:
        spinnaker_helpers.set_enum(node_map, "ExposureAuto", "Off")
        return spinnaker_helpers.try_set_float(node_map, "ExposureTime", exposure_time)
    else:
        spinnaker_helpers.set_enum(node_map, "ExposureAuto", "Continuous")


def set_gain(camera : PySpin.Camera, gain, info):
    """Set the cameras exposure time the the given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if gain > 0:
        spinnaker_helpers.set_enum(node_map, "GainAuto", "Off")
        return spinnaker_helpers.try_set_float(node_map, "Gain", gain)
    else:
        spinnaker_helpers.set_enum(node_map, "GainAuto", "Continuous")


def set_balance_ratio(camera : PySpin.Camera, balance_ratio, info):
    node_map = camera.GetNodeMap()
    if balance_ratio >= 0.5:
        spinnaker_helpers.set_enum(node_map, "BalanceWhiteAuto", "Off")
        return spinnaker_helpers.try_set_float(node_map, "BalanceRatio", balance_ratio)
    else:
        spinnaker_helpers.set_enum(node_map, "BalanceWhiteAuto", "Continuous")

def set_grey_value(camera : PySpin.Camera, value, info):
    """Set the target grey value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if value > 0:
        spinnaker_helpers.set_enum(node_map, "AutoExposureTargetGreyValueAuto", "Off")
        return spinnaker_helpers.try_set_float(node_map, "AutoExposureTargetGreyValue", value)
    else:
        spinnaker_helpers.set_enum(node_map, "AutoExposureTargetGreyValueAuto", "Continuous")


def set_black_level(camera : PySpin.Camera, value, info):
    """Set the target grey value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    return spinnaker_helpers.try_set_float(node_map, "BlackLevel", value)


def set_ev_comp(camera : PySpin.Camera, value, info):
    """Set the EV Compensation"""
    node_map = camera.GetNodeMap()
    return spinnaker_helpers.try_set_float(node_map, "AutoExposureEVCompensation", value)


def set_gamma(camera : PySpin.Camera, value, info):
    """Set the gamma value. If 0 set to off"""
    node_map = camera.GetNodeMap()
    if value > 0:
        spinnaker_helpers.set_bool(node_map, "GammaEnable", True)
        return spinnaker_helpers.try_set_float(node_map, "Gamma", value)
    else:
        spinnaker_helpers.set_bool(node_map, "GammaEnable", False)
        

def set_binning(camera : PySpin.Camera, value : int, info):
    """Set the gamma value. If 0 set to off"""
    node_map = camera.GetNodeMap()
    spinnaker_helpers.try_set_int(node_map, "BinningHorizontal", value)
    spinnaker_helpers.try_set_int(node_map, "BinningVertical", value)

    w = spinnaker_helpers.get_int(node_map, "WidthMax")
    h = spinnaker_helpers.get_int(node_map, "HeightMax")

    spinnaker_helpers.try_set_int(node_map, "Width", w)
    spinnaker_helpers.try_set_int(node_map, "Height", h)




def set_framerate(camera : PySpin.Camera, value : float, info):
  node_map = camera.GetNodeMap()

  if info.is_free_running:
    spinnaker_helpers.try_set_bool(node_map, "AcquisitionFrameRateEnable", True)
    spinnaker_helpers.try_set_float(node_map, "AcquisitionFrameRate", value)

  else:
    spinnaker_helpers.try_set_bool(node_map, "AcquisitionFrameRateEnable", False)


property_setters = dict(
    exposure = set_exposure,
    balance_ratio = set_balance_ratio,
    gain = set_gain,
    grey_value = set_grey_value,
    black_level = set_black_level,
    ev_comp = set_ev_comp,
    gamma = set_gamma,
    max_framerate = set_framerate
)

delayed_setters = dict(    
    binning = set_binning
)