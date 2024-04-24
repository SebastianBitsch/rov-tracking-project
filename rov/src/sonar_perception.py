#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Header, Float32
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState


class SonarLocator():

    def __init__(self):
        """ """
        print("Initialized Sonar point cloud object locator")

        self.rate = rospy.Rate(1)  # 1 Hz
        self.location_publisher = rospy.Publisher('/rov/sonar_point_cloud', PointCloud2, queue_size=10)
        self.distance_publisher = rospy.Publisher('rov/sonar_distance', Float32, queue_size=10)
        self.sonar_subscriber = rospy.Subscriber("/bluerov2/sonar_forward", LaserScan, self.sonar_callback)

        rospy.spin()


    def point_cloud(self, points, parent_frame: str):
        """ """
        itemsize = np.dtype(np.float32).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(np.float32).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate, etc.
        fields = [PointField(name=n, offset=i*itemsize, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]

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
        """
        We start at the center of the sonar and loop outwards, this is done since we 
        assume that the camera is already centered on the object when we use the sonar
        """ 
        ranges = np.asarray(scan.ranges) # 100 rays

        points = []

        # The shittiest code you have ever seen is because we want to start at the middle of the sonar and loop outwards
        # to generate the pattern: 0, -1, 1, -2, 2, -3, 3, -4, ... -180
        indicies = [i // 2 * (i % 2 * 2 - 1) for i in range(len(ranges) + 1)][1:]
        middle_index = len(ranges) // 2 - 1  # i.e. 49

        for i in indicies:
            index = middle_index + i

            # If we hit nothing continue        
            if np.isinf(ranges[index]):
                continue
            
            # Return the distance to the closest hit
            if len(points) == 0:
                # print(f"Found closest point at offset={i}, index={index}, distance={ranges[index]}")
                self.distance_publisher.publish(Float32(data = ranges[index]))
            
            # Get the coordinate of the point
            angle = scan.angle_min + index * scan.angle_increment
            x = ranges[index] * np.cos(angle)
            y = ranges[index] * np.sin(angle)
            points.append([x, y, 0])

        # Turn points into a point cloud and publish them
        self.location_publisher.publish(self.point_cloud(np.array(points), "bluerov2/sonarforward_link"))
        # self.rate.sleep() - this slows everything down - dont know why, therefore commented out


if __name__ == '__main__':
    rospy.init_node('sonar_perception', anonymous=True)
    
    try:
        sl = SonarLocator()
    except rospy.ROSInterruptException:
        pass