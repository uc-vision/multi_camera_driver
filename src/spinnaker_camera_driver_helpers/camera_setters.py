from dataclasses import dataclass
import PySpin
import rospy

from spinnaker_camera_driver_helpers.common import CameraSettings
from . import spinnaker_helpers


def get_config(camera : PySpin.Camera):
    node_map = camera.GetNodeMap()
    return dict(
        exposure=spinnaker_helpers.try_get_value(node_map, "ExposureTime", None),
        gain=spinnaker_helpers.try_get_value(node_map, "Gain", None),    
        balance_ratio=spinnaker_helpers.try_get_value(node_map, "BalanceRatio", None),

        free_running = spinnaker_helpers.get_value(node_map, "AcquisitionFrameRateEnable"),
        max_framerate = spinnaker_helpers.get_value(node_map, "AcquisitionFrameRate"),

        binning = spinnaker_helpers.get_value(node_map, "BinningHorizontal")      
    )


def set_exposure(camera : PySpin.Camera,  config:dict, info:CameraSettings):
    """Set the cameras exposure time to the given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    spinnaker_helpers.try_set_float(node_map, "ExposureTime", config["exposure"])


def set_gain(camera : PySpin.Camera,  config:dict, info:CameraSettings):
    """Set the cameras gain to given value. If 0 set to auto"""
    node_map = camera.GetNodeMap()
    spinnaker_helpers.try_set_float(node_map, "Gain", config["gain"])


def set_balance_ratio(camera : PySpin.Camera, config:dict, info:CameraSettings):
    node_map = camera.GetNodeMap()
    spinnaker_helpers.try_set_float(node_map, "BalanceRatio", config["balance_ratio"])
        

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

    w = spinnaker_helpers.get_value(node_map, "WidthMax")
    h = spinnaker_helpers.get_value(node_map, "HeightMax")

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
    balance_ratio = set_balance_ratio,
    gain = set_gain,

    max_framerate = set_framerate,
    binning = set_binning
)

