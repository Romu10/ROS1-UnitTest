#! /usr/bin/env python3

from course_web_dev_ros.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction, WaypointActionActionGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations
import math
import actionlib

import rospy
import rosunit
import unittest
import rostest
import time
PKG = 'tortoisebot_waypoints'
NAME = 'tortoisebot_waypoints_rotation_test'

class TestRobotWaypoints(unittest.TestCase):

    def setUp(self):

        rospy.init_node('roation_test_node')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.yaw_precision = 0.20
        self.init_position = Point()
        self.current_position = Point()
        self.final_positon = Point()
        # self.get_desired_coordinates()
        self.service_call_world_reset()
        self.get_init_position()
        self.action_service_call()
    
    def get_desired_coordinates(self):
        self.x_coordinates = float(input("Desired X position: "))
        self.y_coordinates = float(input("Desired Y position: "))
        print("Goal coordinates: \nX: %f \nY: %f", self.x_coordinates, self.y_coordinates)
        print(rospy.get_caller_id(), "Goal coordinates")
        

    def get_init_position(self):

        rospy.wait_for_message("/odom", Odometry, timeout=10)
        self.init_position = self.current_position

    def odom_callback(self, msg):

        self.current_position = msg.pose.pose.position

        # yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def feedback_cb(self, msg):
        # print ('Feedback position received:', msg.position)
        self.final_positon = msg.position

    def action_service_call(self):

        client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)

        client.wait_for_server()

        self.goal_msg = Pose()
        self.goal_msg.position.x = 0.1
        self.goal_msg.position.y = 0.1
        self.goal_msg.position.z = 0.0

        client.send_goal(self.goal_msg, feedback_cb=self.feedback_cb)

        client.wait_for_result()

        result = client.get_result()

        return result


    def service_call_world_reset(self):

        rospy.wait_for_service('/gazebo/reset_world')
        s = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        resp = s.call(EmptyRequest())

    def test_correct_position(self):
        #Compare the final orientation with the current position 
        desired_yaw = math.atan2(self.goal_msg.position.y - self.final_positon.y, self.goal_msg.position.x - self.final_positon.x)
        err_yaw = desired_yaw - self.yaw
        print("This is the final error: %f", math.fabs(err_yaw))
        self.assertTrue((math.fabs(err_yaw) < self.yaw_precision), "Orientation error. Minimal orientation error (0.15) was not between the expected values.")


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestRobotWaypoints)
