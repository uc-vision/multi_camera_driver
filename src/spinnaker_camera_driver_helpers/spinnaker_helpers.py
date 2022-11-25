
from typing import Dict, List
import PySpin
import statistics
import rospy

from disable_gc import disable_gc

import gc
from fuzzywuzzy import process



def suggest_node(nodemap, k, threshold=50):
  names = [node.GetName() for node in nodemap.GetNodes()]
  if k in names:
    return "Node {} has wrong type".format(k)

  nearest, score = process.extractOne(k, names)
  suggest = "" if score < threshold else ", did you mean '{}' ({})?".format(nearest, score)
  return 'Node not available {}'.format(k, suggest)


class NodeException(RuntimeError):
  def __init__(self, msg):
    super(NodeException, self).__init__(msg)
    


def get_writable(nodemap, node_name, ptr):
  node = ptr(nodemap.GetNode(node_name))
  if not PySpin.IsAvailable(node):
    raise NodeException(suggest_node(nodemap, node_name))

  # if not PySpin.IsWritable(node):
  #   raise NodeException('Node not writable {}. '.format(node_name))
  return node



def get_readable(nodemap, node_name, ptr):
  node = ptr(nodemap.GetNode(node_name))
  if not PySpin.IsAvailable(node):
    raise NodeException(suggest_node(nodemap, node_name))

  if not PySpin.IsReadable(node):
    raise NodeException('Node not readable {}. '.format(node_name))
  return node

def get_value(nodemap, node_name, ptr):
  node = get_readable(nodemap, node_name, ptr)
  return node.GetValue()

def set_enum(nodemap, node_name, value):
    node = get_writable(nodemap, node_name, PySpin.CEnumerationPtr)

    # Retrieve entry node from enumeration node
    entry = node.GetEntryByName(value)
    if not PySpin.IsAvailable(entry):
        raise NodeException('Entry not available {} - {}. '.format(node_name, value))

    if not PySpin.IsReadable(entry):
        raise NodeException('Entry not readable {} - {} '.format(node_name, value))


    # Retrieve integer value from entry node
    value = entry.GetValue()

    # Set integer value from entry node as new value of enumeration node
    node.SetIntValue(value)


def get_enum(nodemap, node_name):
    node = get_readable(nodemap, node_name, PySpin.CEnumerationPtr)
    # Set integer value from entry node as new value of enumeration node
    return node.GetCurrentEntry().GetSymbolic()

def get_float(nodemap, node_name):
    return get_value(nodemap, node_name, PySpin.CFloatPtr)

def get_int(nodemap, node_name):
    return get_value(nodemap, node_name, PySpin.CIntegerPtr)

def get_bool(nodemap, node_name):
    return get_value(nodemap, node_name, PySpin.CBooleanPtr)



def set_value(nodemap, node_name, value, ptr):
    node = get_writable(nodemap, node_name, ptr)
    node.SetValue(value)
    return value


def set_bool(nodemap, node_name, value):
  return set_value(nodemap, node_name, value, PySpin.CBooleanPtr)


def set_float(nodemap, node_name, value):
  return set_value(nodemap, node_name, value, PySpin.CFloatPtr)

def set_int(nodemap, node_name, value):
  return set_value(nodemap, node_name, value, PySpin.CIntegerPtr)

def try_set_float(nodemap, node_name, value):
    try:
        return set_float(nodemap, node_name, value)
    except PySpin.SpinnakerException as e:
        rospy.loginfo(f"try_set_float {node_name} {value}: {e}")
        return get_float(nodemap, node_name)

def try_set_int(nodemap, node_name, value):
    try:
        return set_int(nodemap, node_name, value)
    except PySpin.SpinnakerException as e:
        rospy.loginfo(f"try_set_int {node_name} {value}: {e}")
        return get_int(nodemap, node_name)

def try_set_bool(nodemap, node_name, value):
    try:
        return set_bool(nodemap, node_name, value)
    except PySpin.SpinnakerException as e:
        rospy.loginfo(f"try_set_bool {node_name} {value}: {e}")
        return get_bool(nodemap, node_name)


@disable_gc
def camera_time_offset(cam, iters=250):
    """ Gets timestamp offset in seconds from input camera """

    # This method is required because the timestamp stored in the camera is relative to when it was powered on, so an
    # offset needs to be applied to get it into epoch time; from tests I've done, this appears to be accurate to ~1e-3
    # seconds.

    timestamp_offsets = []
    for i in range(iters):
        # Latch timestamp. This basically "freezes" the current camera timer into a variable that can be read with
        # TimestampLatchValue()
        cam.TimestampLatch.Execute()

        # Compute timestamp offset in seconds; note that timestamp latch value is in nanoseconds
        timestamp_offset = rospy.get_time() - cam.TimestampLatchValue.GetValue()/1e9

        # Append
        timestamp_offsets.append(timestamp_offset)

    # Return the median value
    return statistics.median(timestamp_offsets)



def activate_image_chunks(nodemap):
    try:
        result = True

        # Activate chunk mode
        #
        # *** NOTES ***
        # Once enabled, chunk data will be available at the end of the payload
        # of every image captured until it is disabled. Chunk data can also be
        # retrieved from the nodemap.
        chunk_mode_active = PySpin.CBooleanPtr(nodemap.GetNode('ChunkModeActive'))

        if PySpin.IsAvailable(chunk_mode_active) and PySpin.IsWritable(chunk_mode_active):
            chunk_mode_active.SetValue(True)
        else:
            rospy.logerr("Couldn't activate chunk mode")

        rospy.logdebug('Chunk mode activated...')

        # Enable all types of chunk data
        #
        # *** NOTES ***
        # Enabling chunk data requires working with nodes: "ChunkSelector"
        # is an enumeration selector node and "ChunkEnable" is a boolean. It
        # requires retrieving the selector node (which is of enumeration node
        # type), selecting the entry of the chunk data to be enabled, retrieving
        # the corresponding boolean, and setting it to be true.
        #
        # In this example, all chunk data is enabled, so these steps are
        # performed in a loop. Once this is complete, chunk mode still needs to
        # be activated.
        chunk_selector = PySpin.CEnumerationPtr(nodemap.GetNode('ChunkSelector'))

        if not PySpin.IsAvailable(chunk_selector) or not PySpin.IsReadable(chunk_selector):
            rospy.logwarn('Unable to retrieve chunk selector. \n')
            return False

        # Retrieve entries
        #
        # *** NOTES ***
        # PySpin handles mass entry retrieval in a different way than the C++
        # API. Instead of taking in a NodeList_t reference, GetEntries() takes
        # no parameters and gives us a list of INodes. Since we want these INodes
        # to be of type CEnumEntryPtr, we can use a list comprehension to
        # transform all of our collected INodes into CEnumEntryPtrs at once.

        entries = [PySpin.CEnumEntryPtr(chunk_selector_entry) for chunk_selector_entry in chunk_selector.GetEntries()]

        rospy.logdebug('Enabling entries...')

        # Iterate through our list and select each entry node to enable
        for chunk_selector_entry in entries:
            # Go to next node if problem occurs
            if not PySpin.IsAvailable(chunk_selector_entry) or not PySpin.IsReadable(chunk_selector_entry):
                continue

            chunk_selector.SetIntValue(chunk_selector_entry.GetValue())

            chunk_symbolic_form = '\t {}:'.format(chunk_selector_entry.GetSymbolic())
            # Retrieve corresponding boolean
            chunk_enable = PySpin.CBooleanPtr(nodemap.GetNode('ChunkEnable'))

            # Enable the boolean, thus enabling the corresponding chunk data
            if not PySpin.IsAvailable(chunk_enable):
                rospy.logwarn('{} not available'.format(chunk_symbolic_form))
                result = False
            # elif chunk_enable.GetValue() is True:
            #     print('{} enabled'.format(chunk_symbolic_form)
            elif PySpin.IsWritable(chunk_enable):
                chunk_enable.SetValue(True)
                rospy.logdebug('{} enabled'.format(chunk_symbolic_form))
            else:
                rospy.logwarn('{} not writable'.format(chunk_symbolic_form))
                result = False

    except PySpin.SpinnakerException as ex:
        rospy.logerr('Error: %s' % ex)
        result = False

    return result


def execute(nodemap, node_name):
    # print("Execute", node_name)
    node = PySpin.CCommandPtr(nodemap.GetNode(node_name))
    if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
        rospy.logerr('Unable to execute {}.  {} {}'.format(node_name, PySpin.IsAvailable(node),
                                                               PySpin.IsWritable(node)))
        return False
    node.Execute()


def reset_camera(camera):
    camera.Init()
    nodemap = camera.GetNodeMap()

    # This often just freezes on re-runs, and everything seems to be OK without it. Is it necessary?
    execute(nodemap, "DeviceReset")  
    camera.DeInit()

def load_defaults(camera):
    nodemap = camera.GetNodeMap()

    set_enum(nodemap, "UserSetSelector", "Default")
    execute(nodemap, "UserSetLoad")


def _reset_all(system):
  camera_list = system.GetCameras()
  rospy.loginfo("Detected {} cameras".format(len(camera_list)))
  cameras = {get_camera_serial(camera):camera for camera in camera_list}


  for k, camera in cameras.items():
      rospy.loginfo("Init {}".format(k))
      camera.Init()
      load_defaults(camera)

  for k, camera in cameras.items():
      rospy.loginfo("Reset {}".format(k))
      nodemap = camera.GetNodeMap()
      execute(nodemap, "DeviceReset")  

  camera_list.Clear()
      

def reset_all():
  rospy.loginfo("Reset all:")
  system = PySpin.System.GetInstance()
  _reset_all(system)
  gc.collect(generation=0)

  r = rospy.Rate(10)
  for i in range(20):
    gc.collect(generation=0)
    r.sleep()

  rospy.loginfo("Release system:")
  system.ReleaseInstance()
  rospy.sleep(1.0)


  rospy.loginfo("Done")


def load_defaults(camera):
    rospy.loginfo("Loading defaults")
    nodemap = camera.GetNodeMap()
    set_enum(nodemap, "UserSetSelector", "Default")
    execute(nodemap, "UserSetLoad")

def trigger(camera):
    nodemap = camera.GetNodeMap()
    execute(nodemap, "TriggerSoftware")


def trigger_slave(camera : PySpin.Camera):
    nodemap = camera.GetNodeMap()

    set_enum(nodemap, "LineSelector", "Line3")
    set_enum(nodemap, "TriggerSource", "Line3")
    set_enum(nodemap, "TriggerSelector", "FrameStart")
    set_enum(nodemap, "LineMode", "Input")
    set_enum(nodemap, "TriggerOverlap", "ReadOut")
    set_enum(nodemap, "TriggerActivation", "RisingEdge")
    set_enum(nodemap, "TriggerMode", "On")


def trigger_master(camera : PySpin.Camera, free_running : bool):
  nodemap = camera.GetNodeMap()

  set_enum(nodemap, "LineSelector", "Line2")
  set_enum(nodemap, "LineMode", "Output")
  set_enum(nodemap, "TriggerSource", "Software")

  if not free_running:
    set_enum(nodemap, "TriggerMode", "On")



def set_setting(nodemap, setting:Dict):
  try:

    setting_name = next(iter(setting)) # setting.keys()[0]
    value, type_ = setting[setting_name]
    if type_ == "ENUM":
        set_enum(nodemap, setting_name, value)
        if setting == "UserSetSelector":
            execute(nodemap, "UserSetLoad")
    elif type_ == "BOOL":
        set_bool(nodemap, setting_name, value)
    elif type_ == "CHUNK":
        activate_image_chunks(nodemap)
    elif type_ == "FLOAT":
        set_float(nodemap, setting_name, value)
    elif type_ == "INT":
        set_int(nodemap, setting_name, value)

    else:
        rospy.logerr("Unknown type {} for {}".format(type_, setting_name))
  except (NodeException, PySpin.SpinnakerException) as e:
    rospy.logerr("Exception setting {}: {}".format(setting_name, str(e)))


def set_settings(nodemap, settings:List[Dict]):
  for setting in settings:
      set_setting(nodemap, setting)



def set_camera_settings(camera, settings):

    nodemap = camera.GetNodeMap()
    set_settings(nodemap, settings["device"])


    s_node_map = camera.GetTLStreamNodeMap()
    set_settings(s_node_map, settings["transport_layer"])

def get_camera_encoding(camera):
    nodemap = camera.GetNodeMap()
    return get_enum(nodemap, "PixelFormat")

def get_current_speed(camera):
    d_node_map = camera.GetTLDeviceNodeMap()
    return get_enum(d_node_map, "DeviceCurrentSpeed")
 

def get_camera_serial(cam):
    nodemap_tldevice = cam.GetTLDeviceNodeMap()
    node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
    device_serial_number = node_device_serial_number.GetValue()
    return int(device_serial_number)


def camera_list_to_dict(camera_list):
    camera_dict = {}
    for cam in camera_list:
        camera_dict[get_camera_serial(cam)] = cam
    return camera_dict

def find_cameras(camera_serials):
    camera_list = PySpin.System.GetInstance().GetCameras()
    serial_dict = camera_list_to_dict(camera_list)

    cameras = {}
    for serial, alias in camera_serials.items():
        if serial not in serial_dict:
            rospy.logerr(f"Could not find camera {serial} : {alias}")

        cameras[alias] = serial_dict[serial]

    camera_list.Clear()
    return cameras


def validate_init(camera):
    return camera.IsValid() and camera.IsInitialized()


def validate_streaming(camera):
    return validate_init(camera) and camera.IsStreaming()


def get_image_size(camera : PySpin.Camera):
    node_map = camera.GetNodeMap()

    w = get_int(node_map, "Width")
    h = get_int(node_map, "Height")

    return (w, h)
