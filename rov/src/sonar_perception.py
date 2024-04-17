#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState


class SonarLocator():

    def __init__(self):
        """ """
        print("Initialized Sonar point cloud object locator")

        rate = rospy.Rate(10)  # 10 Hz
        self.location_publisher = rospy.Publisher('/rov/sonar_point_cloud', PointCloud2, queue_size=10)
        self.sonar_subscriber = rospy.Subscriber("/bluerov2/sonar_forward", LaserScan, self.sonar_callback)

        rospy.spin()


    def point_cloud(self, points, parent_frame: str):
        """ """
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate('xyz')]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = Header(frame_id=parent_frame)

        return PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of three float32s.
            row_step=(itemsize * 3 * points.shape[0]), 
            data=data
        )

    def sonar_callback(self, scan: LaserScan):
        """ """ 
        ranges      = np.asarray(scan.ranges)
        intensities = np.asarray(scan.intensities)

        points = []

        # for angle in np.linspace(scan.angle_min, scan.angle_max, num = scan.)
        for i in range(len(ranges)):

            # If we hit nothing continue        
            if np.isinf(ranges[i]):
                continue

            angle = scan.angle_min + i * scan.angle_increment
            x = ranges[i] * np.cos(angle)
            y = ranges[i] * np.sin(angle)
            points.append([x,y,0])

        self.location_publisher.publish(self.point_cloud(np.array(points), "bluerov2/sonarforward_link"))


if __name__ == '__main__':

    # model_name = rospy.get_param('~model_name', "ooi")
    rospy.init_node('sonar_perception', anonymous=True)
    
    try:
        print("JKNAKSJD")
        sl = SonarLocator()
    except rospy.ROSInterruptException:
        pass