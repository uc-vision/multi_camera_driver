#!/usr/bin/env python3

"""ROS node to reset all Spinnaker based cameras found on a system.

Copyright (c) 2019. UC Vision, Canterbury University. This file is subject to the 3-clause BSD
license, as found in the LICENSE file in the top-level directory of this
distribution and at https://github.com/sds53/spinnaker_camera_driver_ros/LICENSE.
No part of multi_camera_driver.helpers, including this file, may be copied, modified,
propagated, or distributed except according to the terms contained in the
LICENSE file.
"""

import rospy2 as rospy
from multi_camera_driver.helpers import spinnaker_helpers
from multi_camera_driver.helpers.config import exceptions_to_rosout

def main():
    exceptions_to_rosout()
    rospy.init_node('camera_array_node', anonymous=False)
    spinnaker_helpers.reset_all()


if __name__ == "__main__":
    main()
