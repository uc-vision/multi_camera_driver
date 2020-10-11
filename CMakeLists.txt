cmake_minimum_required(VERSION 2.8.3)
project(spinnaker_camera_driver_ros)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  cv_bridge
)

catkin_python_setup()


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/CameraArraySettings.cfg
   cfg/CameraIMUSettings.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera_focus_tool
#  CATKIN_DEPENDS dynamic_reconfigure rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
