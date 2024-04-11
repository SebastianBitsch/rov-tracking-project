#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState

from commons.spawn_object import spawn_object

def move_straight(model_name: str, speed: float, axis: str):
    """
    Moves a specified Gazebo model straight along the specified axis at a given speed.

    Args:
        model_name (str, optional): The name of the Gazebo model to be moved.
        speed (float, optional): The speed at which the model moves along the specified axis.
        axis (str, optional): The axis along which the model moves. Can be 'x', 'y', or 'z'.

    Note:
        This function publishes the model's new state to the '/gazebo/set_model_state' topic in the Gazebo environment.

    Example:
        To move a model named 'robot' along the y-axis at a speed of 0.2:
        >>> move_straight(model_name="robot", speed=0.2, axis="y")
    """
    rospy.init_node('move_straight', anonymous=True)

    rate = rospy.Rate(10)  # 10 Hz
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    
    state_msg = ModelState(model_name, Pose(Point(), Quaternion()), Twist(), "world")

    step = 0
    while not rospy.is_shutdown():

        if axis == 'x':
            state_msg.pose.position.x = speed * step
        elif axis == 'y':
            state_msg.pose.position.y = speed * step
        else:
            state_msg.pose.position.z = speed * step
            
        pub.publish(state_msg)
        rate.sleep()
        step += 1


if __name__ == '__main__':

    model_name = rospy.get_param('~model_name', "ooi")
    model_pos = rospy.get_param('~model_pos', [0, 0, 0])
    model_rot = rospy.get_param('~model_rot', [0, 0, 0, 1])
    movement_axis = rospy.get_param('~move_axis', 'x')
    movement_speed = rospy.get_param('~speed', 0.1)

    try:
        # Spawn object
        spawn_object(model_name = model_name, position = model_pos, rotation = model_rot)
        
        print("Spawned object, now moving")

        # Move object
        move_straight(model_name = model_name, speed = movement_speed, axis = movement_axis)

    except rospy.ROSInterruptException:
        pass