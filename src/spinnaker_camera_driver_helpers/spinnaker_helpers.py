from __future__ import print_function

import PySpin
import rospy


def set_enum(nodemap, node_name, value):
    node = PySpin.CEnumerationPtr(nodemap.GetNode(node_name))
    if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
        print('Unable to set {} to {} (enum retrieval). Aborting...'.format(node, value))
        return False

    # Retrieve entry node from enumeration node
    entry = node.GetEntryByName(value)
    if not PySpin.IsAvailable(entry) or not PySpin.IsReadable(
            entry):
        print('Unable to set {} to {} (entry retrieval). Aborting...'.format(node_name, value))
        return False

    # Retrieve integer value from entry node
    value = entry.GetValue()

    # Set integer value from entry node as new value of enumeration node
    node.SetIntValue(value)


def get_enum(nodemap, node_name):
    node = PySpin.CEnumerationPtr(nodemap.GetNode(node_name))
    if not PySpin.IsAvailable(node) or not PySpin.IsReadable(node):
        print('Unable to read {} (enum retrieval). Aborting...'.format(node_name))
        return False

    # Set integer value from entry node as new value of enumeration node
    return node.GetCurrentEntry().GetSymbolic()

def get_float(nodemap, node_name):
    node = PySpin.CFloatPtr(nodemap.GetNode(node_name))
    if not PySpin.IsAvailable(node) or not PySpin.IsReadable(node):
        print('Unable to read {} (float retrieval). Aborting...'.format(node_name))
        return False

    # Set integer value from entry node as new value of enumeration node
    return node.GetValue()

def set_bool(nodemap, node_name, value):
    node = PySpin.CBooleanPtr(nodemap.GetNode(node_name))
    if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
        print('Unable to set {} to {} (enum retrieval). Aborting...'.format(node_name, value))
        return False

    # Set integer value from entry node as new value of enumeration node
    node.SetValue(value)


def set_float(nodemap, node_name, value):
    node = PySpin.CFloatPtr(nodemap.GetNode(node_name))
    if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
        print(PySpin.IsAvailable(node), PySpin.IsWritable(node))
        print('Unable to set {} to {} (node map retrieval). Aborting...'.format(node_name, value))
        return False

    # Set integer value from entry node as new value of enumeration node
    node.SetValue(value)


def set_int(nodemap, node_name, value):
    node = PySpin.CIntegerPtr(nodemap.GetNode(node_name))
    if not PySpin.IsAvailable(node) or not PySpin.IsWritable(node):
        print(PySpin.IsAvailable(node), PySpin.IsWritable(node))
        print('Unable to set {} to {} (node map retrieval). Aborting...'.format(node_name, value))
        return False

    # Set integer value from entry node as new value of enumeration node
    node.SetValue(value)



def activate_image_chunks(nodemap):
    try:
        result = True
        rospy.logdebug('\n*** CONFIGURING CHUNK DATA ***\n')

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
            print("Couldn't activate chunk mode")

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
            rospy.logwarn('Unable to retrieve chunk selector. Aborting...\n')
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
        print('Unable to execute {}. Aborting... {} {}'.format(node_name, PySpin.IsAvailable(node),
                                                               PySpin.IsWritable(node)))
        return False
    node.Execute()


def reset_camera(camera):
    camera.Init()
    nodemap = camera.GetNodeMap()
    set_enum(nodemap, "UserSetSelector", "Default")
    execute(nodemap, "UserSetLoad")

    # This often just freezes on re-runs, and everything seems to be OK without it. Is it necessary?
    # execute(nodemap, "DeviceReset")  
    camera.DeInit()

def reset_all():
    print("Reset all:")
    system = PySpin.System.GetInstance()
    camera_list = system.GetCameras()
    camera = None

    print("Detected {} cameras".format(len(camera_list)))
    for i, camera in enumerate(camera_list):
        nodemap_tldevice = camera.GetTLDeviceNodeMap()
        serial = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
        print("Reset: ", serial.GetValue())
        reset_camera(camera)

    print("Done")

    del camera
    camera_list.Clear()

    print("Release system:")
    system.ReleaseInstance()


def load_defaults(camera):
    print("Loading defaults")
    nodemap = camera.GetNodeMap()
    set_enum(nodemap, "UserSetSelector", "Default")
    execute(nodemap, "UserSetLoad")
    # execute(nodemap, "DeviceReset")


def trigger(camera):
    nodemap = camera.GetNodeMap()
    execute(nodemap, "TriggerSoftware")


def enable_triggering(camera, master=True):
    nodemap = camera.GetNodeMap()

    if master:
        print("MASTER----------------")
        set_enum(nodemap, "LineSelector", "Line2")
        set_enum(nodemap, "LineMode", "Output")
        set_enum(nodemap, "TriggerSource", "Software")
    else:
        set_enum(nodemap, "LineSelector", "Line3")
        set_enum(nodemap, "TriggerSource", "Line3")
        set_enum(nodemap, "TriggerSelector", "FrameStart")
        set_enum(nodemap, "LineMode", "Input")
        set_enum(nodemap, "TriggerOverlap", "ReadOut")
        set_enum(nodemap, "TriggerActivation", "RisingEdge")

    set_enum(nodemap, "TriggerMode", "On")

    # set_bool(nodemap, "DeviceReset", True)


def set_settings(nodemap, settings):
    for setting in settings:

        setting_name = setting.keys()[0]
        value, type_ = setting[setting_name]
        #print(setting_name,value, type_)
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
            print(type_, "Unknown, not setting")


def set_camera_settings(camera, settings):
    nodemap = camera.GetNodeMap()
    set_settings(nodemap, settings["device"])

    s_node_map = camera.GetTLStreamNodeMap()
    set_settings(s_node_map, settings["transport_layer"])

    d_node_map = camera.GetTLDeviceNodeMap()
    print(get_enum(d_node_map, "DeviceCurrentSpeed"))


def camera_list_to_dict(camera_list):
    camera_dict = {}
    for cam in camera_list:
        nodemap_tldevice = cam.GetTLDeviceNodeMap()
        node_device_serial_number = PySpin.CStringPtr(nodemap_tldevice.GetNode('DeviceSerialNumber'))
        device_serial_number = node_device_serial_number.GetValue()
        camera_dict[int(device_serial_number)] = cam
    return camera_dict
