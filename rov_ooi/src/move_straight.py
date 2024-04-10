#!/usr/bin/env python

import rospy
import rospkg

from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import time

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *

import numpy as np

def spawn_object(model_name: str = "ooi", position: np.ndarray = np.zeros(3), rotation: np.ndarray = np.array([0,0,0,1])) -> None: 
    """
    Spawn an object in Gazebo environment.

    Args:
        model_name (str): The name of the model to spawn.
        position (np.ndarray, optional): The position of the object in the world frame. Defaults to np.zeros(3).
        rotation (np.ndarray, optional): The rotation of the object represented as a quaternion [x, y, z, w]. Defaults to np.array([0,0,0,1]).

    Raises:
        AssertionError: If position is not a 3-element array or rotation is not a 4-element array.

    Note:
        This function requires ROS and Gazebo to be running.
    """
    position = np.array(position)
    rotation = np.array(rotation)
    assert position.shape[0] == 3 and rotation.shape[0] == 4, "Error: position should be a vec3f and rotation a vec4f"

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


def move_straight(model_name: str = "ooi", speed: float = 0.1, axis: str = "x"):
    """
    Moves a specified Gazebo model straight along the specified axis at a given speed.

    Args:
        model_name (str, optional): The name of the Gazebo model to be moved. Defaults to "ooi".
        speed (float, optional): The speed at which the model moves along the specified axis. Defaults to 0.1.
        axis (str, optional): The axis along which the model moves. Can be 'x', 'y', or 'z'. Defaults to "x".

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