cmake_minimum_required(VERSION 2.8.3)
project(spinnaker_camera_driver_ros)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
  cv_bridge
  message_generation
  diagnostic_updater
)

catkin_python_setup()


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/CameraArray.cfg
)


file(GLOB msg_files "msg/*.msg")
foreach (file ${msg_files})
    get_filename_component(file_short ${file} NAME)
    list(APPEND msg_base_files ${file_short})
endforeach ()

add_message_files(DIRECTORY msg FILES ${msg_base_files})
message(STATUS "+++ Messages: ${msg_base_files}")

## In order to add services, adding .srv files in srv/ is enough
## Don't need to modify following code.
file(GLOB srv_files "srv/*.srv")
foreach (file ${srv_files})
    get_filename_component(file_short ${file} NAME)
    list(APPEND srv_base_files ${file_short})
endforeach ()

add_service_files(DIRECTORY srv FILES ${srv_base_files})
message(STATUS "+++ Services: ${srv_base_files}")

generate_messages(
  DEPENDENCIES
  std_msgs
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
