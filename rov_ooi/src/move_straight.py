#!/usr/bin/env python

import rospy
import rospkg

from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import time

from gazebo_msgs.srv import *

import numpy as np

def spawn_object(model_name: str = "ooi", position: np.ndarray = np.zeros(3), rotation: np.ndarray = np.array([0,0,0,1])): 
    """ """

    model_pose = Pose(
        Point(
            x = position[0],
            y = position[1],
            z = position[2],
        ), 
        Quaternion(
            x = rotation[0],
            y = rotation[1],
            z = rotation[2],
            w = rotation[3]
        )
    )

    # "SpawnModel" is a Gazebo built in message we can write to 
    # See: http://docs.ros.org/en/noetic/api/gazebo_msgs/html/srv/SpawnModel.html
    spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    destroyer = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel) # Not used

    # Get path to file inside this package
    ros_pack_target = rospkg.RosPack()
    urdf_path = ros_pack_target.get_path('rov_ooi')
    urdf_path += f'/src/{model_name}.urdf'

    # Read the model XML file
    with open(urdf_path, "r") as g:
        model_xml = g.read()

    # Spawn the model in the world
    res = spawner.call(
        model_name,     # name of the model
        model_xml,      # xml urdf file of the model to spawn
        "",             # model namespace, not important for us
        model_pose,     # position and rotation of the model
        "world"         # gazebo world frame is used as parent
    )
    print(f"Spawned object {model_name} with result:")
    print(res)

    # We can (could) destroy the model with
    # destroyer.call(model_name)


def heading_publisher():
    rospy.init_node('move_straight', anonymous=True)

    # Get parameters from ROS parameter server
    period = rospy.get_param('~period', 6.0)
    speed = rospy.get_param('~speed', 0.5)

    pub = rospy.Publisher('bluerov2/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        twist_msg = Twist()
        twist_msg.angular.z = speed  # Initial angular velocity

        # Publish initial velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < period/2:
            pub.publish(twist_msg)
            rate.sleep()

        # Change angular velocity to -speed
        twist_msg.angular.z = -speed

        # Publish negative velocity for T/2 seconds
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < period/2:
            pub.publish(twist_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        # heading_publisher()
        spawn_object()
    except rospy.ROSInterruptException:
        pass