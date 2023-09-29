# spinnaker_camera_driver_ros
ROS driver for FLIR cameras using ros


## Install
Download [Spinnaker SDK](https://flir.app.boxcn.net/v/SpinnakerSDK/folder/74727114471) and install following the README.

Download [PySpin](https://flir.app.boxcn.net/v/SpinnakerSDK/folder/74728699483) from the FLIR website and install following the README.


## Mock camera driver (image directory publisher)

Example usage:
'''rosrun spinnaker_camera_driver_ros image_folder.py _image_path:=$HOME/scans/03-03/19-23-00/ _looping:=true _calibration:=my_calibration.json''' 

## Adding GPS based UTC timestamps to images.

The camera driver can publish utc timestamps for published images if a gps disciplined clock is added to the system.
The clock needs to be running the firmware in the directory ```src/firmware/main.py``` and can be installed with ```mpremote cp src/trigger_reporter/firmware.py :main.py```.

Additionally the path to the serial port of the clock needs to be specified in the camera_set params.
```yaml
trigger_reporter: /dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e660d05113943d35-if00
```

The node will now publish an additional ```TimeReference``` topic ```/utc`` under each camera.
```
/camera_array/cam1/camera_info
/camera_array/cam1/color
/camera_array/cam1/color/compressed
/camera_array/cam1/color/preview/compressed
/camera_array/cam1/utc
```
