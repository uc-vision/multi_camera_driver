import rospy
from maara_msgs.msg import RailStatus


class ScannerNode(object):
    def __init__(self):
        pass

    def camera_array_callback(self, image_msg1, image_msg2, image_msg3, image_msg4, image_msg5, image_msg6, status):
        sys_phase = None
        if status == "READY" and sys_phase == "SCANNING":
            pass

    def run(self):
        pass


if __name__ == "__main__":
    rospy.init_node("scanner_node")
    node = ScannerNode()
    node.run()
