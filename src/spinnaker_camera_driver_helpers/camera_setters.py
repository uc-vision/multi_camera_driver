from dataclasses import dataclass
import PySpin
import rospy

from spinnaker_camera_driver_helpers.common import CameraSettings

from . import spinnaker_helpers





def get_config(camera : PySpin.Camera):
    node_map = camera.GetNodeMap()
    return dict(
        exposure=spinnaker_helpers.get_float(node_map, "ExposureTime"),
        auto_exposure=spinnaker_helpers.get_enum(node_map, "ExposureAuto") == "Continuous",

        gain=spinnaker_helpers.get_float(node_map, "Gain"),
        auto_gain=spinnaker_helpers.get_enum(node_map, "GainAuto") == "Continuous",
        
        balance_ratio=spinnaker_helpers.get_float(node_map, "BalanceRatio"),
        auto_white_balance=spinnaker_helpers.get_enum(node_map, "BalanceWhiteAuto") == "Continuous",

        grey_value=spinnaker_helpers.get_float(node_map, "AutoExposureTargetGreyValue"),
        auto_grey_value=spinnaker_helpers.get_enum(node_map, "AutoExposureTargetGreyValueAuto") == "Continuous",

        black_level=spinnaker_helpers.get_float(node_map, "BlackLevel"),
        
        gamma=spinnaker_helpers.get_float(node_map, "Gamma"),
        gamma_enable=spinnaker_helpers.get_bool(node_map, "GammaEnable"),

        free_running = spinnaker_helpers.get_bool(node_map, "AcquisitionFrameRateEnable"),
        max_framerate = spinnaker_helpers.get_float(node_map, "AcquisitionFrameRate"),

        binning = spinnaker_helpers.get_int(node_map, "BinningHorizontal")      
    )


def set_exposure(camera : PySpin.Camera,  config:dict, info:CameraSettings):
    """Set the cameras exposure time to the given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if config["auto_exposure"] == False:
      spinnaker_helpers.set_enum(node_map, "ExposureAuto", "Off")
      spinnaker_helpers.try_set_float(node_map, "ExposureTime", config["exposure"])
    else:
      spinnaker_helpers.set_enum(node_map, "ExposureAuto", "Continuous")


def set_gain(camera : PySpin.Camera,  config:dict, info:CameraSettings):
    """Set the cameras gain to given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    if config["auto_gain"] == False:
      spinnaker_helpers.set_enum(node_map, "GainAuto", "Off")
      spinnaker_helpers.try_set_float(node_map, "Gain", config["gain"])
    else:
        spinnaker_helpers.set_enum(node_map, "GainAuto", "Continuous")

def set_balance_ratio(camera : PySpin.Camera, config:dict, info:CameraSettings):
    node_map = camera.GetNodeMap()
    if config["auto_white_balance"] == False:
      spinnaker_helpers.set_enum(node_map, "BalanceWhiteAuto", "Off")
      spinnaker_helpers.try_set_float(node_map, "BalanceRatioSelector", config["balance_ratio"])
    else:
        spinnaker_helpers.set_enum(node_map, "BalanceWhiteAuto", "Continuous")
        

def set_grey_value(camera : PySpin.Camera,  config:dict, info:CameraSettings):
    """Set the target grey value. If 0 set to auto"""
    node_map = camera.GetNodeMap()

    if config["auto_grey_value"] == False:
      spinnaker_helpers.set_enum(node_map, "AutoExposureTargetGreyValueAuto", "Off")
      spinnaker_helpers.try_set_float(node_map, "AutoExposureTargetGreyValue", config["grey_value"])
    else:
      spinnaker_helpers.set_enum(node_map, "AutoExposureTargetGreyValueAuto", "Continuous")


  
def set_black_level(camera : PySpin.Camera, config:dict, info:CameraSettings):
    """Set the target black level"""
    node_map = camera.GetNodeMap()
    value = spinnaker_helpers.try_set_float(node_map, "BlackLevel", config["black_level"])
    return dict(black_level=value)


def set_ev_comp(camera : PySpin.Camera, config:dict, info):
    """Set the EV Compensation"""
    node_map = camera.GetNodeMap()
    ev_comp = spinnaker_helpers.try_set_float(node_map, "AutoExposureEVCompensation", config["ev_comp"])
    return dict(ev_comp=ev_comp)


def set_gamma(camera : PySpin.Camera, config:dict, info:CameraSettings):
    """Set the gamma value. If 0 set to off"""
    node_map = camera.GetNodeMap()
    
    if config["gamma_enable"] == True:
        spinnaker_helpers.set_bool(node_map, "GammaEnable", True)
        spinnaker_helpers.try_set_float(node_map, "Gamma", config["gamma"])
    else:
        spinnaker_helpers.set_bool(node_map, "GammaEnable", False)

        

def set_binning(camera : PySpin.Camera, config:dict, info:CameraSettings):
    """Set the gamma value. If 0 set to off"""
    node_map = camera.GetNodeMap()

    value = config["binning"]
    spinnaker_helpers.try_set_int(node_map, "BinningHorizontal", value)
    spinnaker_helpers.try_set_int(node_map, "BinningVertical", value)

    w = spinnaker_helpers.get_int(node_map, "WidthMax")
    h = spinnaker_helpers.get_int(node_map, "HeightMax")

    spinnaker_helpers.try_set_int(node_map, "Width", w)
    spinnaker_helpers.try_set_int(node_map, "Height", h)




def set_framerate(camera : PySpin.Camera,  config:dict, info:CameraSettings):
  node_map = camera.GetNodeMap()

  free_running = info.master_id is None or info.master_id == info.name
  spinnaker_helpers.try_set_bool(node_map, "AcquisitionFrameRateEnable", free_running)
  if free_running:
    spinnaker_helpers.try_set_float(node_map, "AcquisitionFrameRate", config["max_framerate"])


property_setters = dict(
    exposure = set_exposure,
    auto_exposure = set_exposure,

    balance_ratio = set_balance_ratio,
    auto_white_balance = set_balance_ratio,

    gain = set_gain,
    auto_gain = set_gain,
    
    grey_value = set_grey_value,
    auto_grey_value = set_grey_value,

    black_level = set_black_level,
    ev_comp = set_ev_comp,
    
    gamma = set_gamma,
    gamma_enable = set_gamma,

    max_framerate = set_framerate,
    binning = set_binning
)

