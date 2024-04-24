#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState


class SensorFusion():

    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('sensor_fusion', anonymous=True)
    
    try:
        sl = SensorFusion()
    except rospy.ROSInterruptException:
        pass