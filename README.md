# spinnaker_camera_driver_ros
ROS driver for FLIR cameras using ros


## Install
Download [Spinnaker SDK](https://flir.app.boxcn.net/v/SpinnakerSDK/folder/74727114471) and install following the README.

Download [PySpin](https://flir.app.boxcn.net/v/SpinnakerSDK/folder/74728699483) from the FLIR website and install following the README.


## Mock camera driver (image directory publisher)

Example usage:
'''rosrun spinnaker_camera_driver_ros image_folder.py _image_path:=$HOME/scans/03-03/19-23-00/ _looping:=true _calibration:=my_calibration.json''' 