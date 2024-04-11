#!/usr/bin/env python

import rospy
import numpy as np

from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState

from commons.spawn_object import spawn_object

def move_circle(model_name: str, speed: float, radius: float):
    """
    Moves a specified Gazebo model in a circle in the x,y-plane

    Args:
        model_name (str, optional): The name of the Gazebo model to be moved
        speed (float, optional): The speed at which the model moves along the specified axis
        radius (float, optional): The radius of the circle to move in

    Note:
        This function publishes the model's new state to the '/gazebo/set_model_state' topic in the Gazebo environment.

    Example:
        To move a model named 'robot' along the at a speed of 0.2:
        >>> move_circle(model_name="robot", speed=0.2)
    """
    rospy.init_node('move_circle', anonymous=True)

    rate = rospy.Rate(10)  # 10 Hz
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    
    state_msg = ModelState(model_name, Pose(Point(), Quaternion()), Twist(), "world")

    step = 0
    while not rospy.is_shutdown():

        angle = np.deg2rad(step * speed)
        
        state_msg.pose.position.x = radius * np.cos(angle)
        state_msg.pose.position.y = radius * np.sin(angle)

        pub.publish(state_msg)
        rate.sleep()
        step += 1


if __name__ == '__main__':

    model_name = rospy.get_param('~model_name', "ooi")
    model_pos = rospy.get_param('~model_pos', [0, 0, 0])
    model_rot = rospy.get_param('~model_rot', [0, 0, 0, 1])
    movement_radius = rospy.get_param('~move_radius', 50.0)
    movement_speed = rospy.get_param('~move_speed', 0.5)

    try:
        # Spawn object
        spawn_object(model_name = model_name, position = model_pos, rotation = model_rot)
        
        print("Spawned object, now moving")

        # Move object
        move_circle(model_name = model_name, speed = movement_speed, radius = movement_radius)

    except rospy.ROSInterruptException:
        pass