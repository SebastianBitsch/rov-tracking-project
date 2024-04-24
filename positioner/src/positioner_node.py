#!/usr/bin/env python

import rospy
from tracker.msg import Tracked_object
from geometry_msgs.msg import Twist
import geometry_msgs.msg as geometry_msgs
from PID import PIDRegulator
import tf.transformations as trans
import numpy
from std_msgs.msg import Float32


class TargetFollower:
    def __init__(self):
        rospy.init_node('target_follower', anonymous=False)

        self.target_position = None

        self.cmd_vel_pub = rospy.Publisher('/bluerov2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/tracker/tracked_object', Tracked_object, self.target_position_callback)
        rospy.Subscriber('/rov/sonar_distance', Float32, self.sonar_distancer_callback)

        self.GOAL_TOLERANCE = 5
        self.LINEAR_GAIN = 3.0
        self.TURNING_GAIN = 0.03

        self.rate = rospy.Rate(10)  # 10 Hz

        self.pid_rot = PIDRegulator(1, 0, 0, 1)
        self.pid_pos = PIDRegulator(1, 0, 0, 1)

        self.pos_des = numpy.zeros(3)
        self.quat_des = numpy.array([0, 0, 0, 1])

        self.dis = 40.0

    def sonar_distancer_callback(self, data):  # call teh sonar data
        self.dis = data.data

    def target_position_callback(self, data):
        self.target_position = data

    def seek_target(self):
        cmd_vel_msg = Twist()
        if self.target_position is not None and self.target_position.pixel_centerX > 1920 / 2:
            cmd_vel_msg.angular.z = -0.2
        elif self.target_position is not None and self.target_position.pixel_centerX < 1920 / 2:
            cmd_vel_msg.angular.z = 0.2

        self.cmd_vel_pub.publish(cmd_vel_msg)

    def position_self(self):

        while not rospy.is_shutdown():

            screen_width = 1920
            screen_height = 1080
            tolerance = 20
            desired_distance = 1000
            t = rospy.get_rostime().secs

            if self.target_position is None or self.target_position.lost is True:
                self.seek_target()
                self.rate.sleep()
                continue

            cmd_vel_msg = Twist()

            error_h_pixels = self.target_position.pixel_centerX - (screen_width / 2 - self.target_position.pixel_width)
            error_h_normalized = normalize(error_h_pixels, -960, 960)

            error_v_pixels = self.target_position.pixel_centerY - (
                    screen_height / 2)
            error_v_normalized = normalize(error_v_pixels, -540, 540)

            error_dist_normalized = 0

            if error_v_normalized + error_h_normalized < 0.1:
                error_dist = desired_distance - self.dis * 1000  # added self.dis which will host the sonar distance from sonar, converted from m to mm
                error_dist_normalized = normalize(error_dist, -80000, 80000)

            # q = msg.pose.pose.orientation
            q = numpy.array([0, 0, error_h_normalized, 1])
            p = numpy.array([error_dist_normalized, 0, error_v_normalized])

            rospy.loginfo(
                "errors: dist: " + str(error_dist_normalized) + " v: " + str(error_v_normalized) + " h: " + str(
                    error_h_normalized))

            e_pos_world = self.pos_des - p
            e_pos_body = trans.quaternion_matrix(q).transpose()[0:3, 0:3].dot(e_pos_world)

            e_rot_quat = trans.quaternion_multiply(trans.quaternion_conjugate(q), self.quat_des)
            e_rot = numpy.array(trans.euler_from_quaternion(e_rot_quat))

            v_linear = self.pid_pos.regulate(e_pos_body, t)
            v_angular = self.pid_rot.regulate(e_rot, t)

            cmd_vel_msg.linear = geometry_msgs.Vector3(*v_linear)
            cmd_vel_msg.angular = geometry_msgs.Vector3(*v_angular)

            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.rate.sleep()


def normalize(val, min_val, max_val, new_min=-1, new_max=1):
    normalized_val = ((val - min_val) / (max_val - min_val)) * (new_max - new_min) + new_min
    return normalized_val


if __name__ == '__main__':
    try:
        target_follower = TargetFollower()
        target_follower.position_self()
    except rospy.ROSInterruptException:
        pass
