#!/usr/bin/env python
# -*- coding: utf-8 -*-

# standard module
import rospy
import tf2_ros
from termcolor import colored

# custom module
from ROS_visualization.ros_visuals import ROSMarkerPublisher
from ROS_visualization.logger import Logger

# ros messages and services
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
from mpc_pkg.srv import GoalInfo, GoalInfoResponse


class goal_receiver_node():
    def __init__(self):
        # parameters
        self.enable_logging = rospy.get_param("goal_receiver_enable_logging", False)
        self.radius = rospy.get_param("radius", 0.5)

        #  attribute
        self.goal_received = False
        self.goal_achieved = False
        self.goal_pose_in_map = PoseStamped()
        self.logger = Logger("goal_receiver", 1)

        # define goal topic subscriber
        self.goal_subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.GoalCB)
        self.visuals = ROSMarkerPublisher('/goal_receiver/visuals', 100)
        self.set_goal_achieved_server = rospy.Service("/goal_receiver/set_goal_achieved", Empty, self.SetGoalAchievedCB)
        self.get_goal_info_server = rospy.Service("/goal_receiver/get_goal_info", GoalInfo, self.GetGoalInfoCB)

    def visualize(self):
        if self.goal_received and not self.goal_achieved:
            circle_pose = Pose()
            circle_pose.position.x = self.goal_pose_in_map.pose.position.x
            circle_pose.position.y = self.goal_pose_in_map.pose.position.y

            self.visual_goal = self.visuals.get_circle("map")
            self.visual_goal.set_scale(0.3 + self.radius, 0.3 + self.radius, 0.01)
            self.visual_goal.set_color(15, 0.5)

            self.visual_goal.add_marker(circle_pose)

        self.visuals.publish()

    def GoalCB(self, msg):
        self.logger.log("started")

        self.goal_pose_in_map = msg
        self.logger.log("goal pose in map:", type=1)
        self.logger.log(self.goal_pose_in_map, type=1)

        self.goal_received = True
        self.goal_achieved = False
        self.logger.log("end")

    def SetGoalAchievedCB(self, req):
        self.goal_achieved = True
        self.goal_received = False

        res = EmptyResponse()
        return res

    def GetGoalInfoCB(self, req):
        res = GoalInfoResponse()
        res.goal_received = self.goal_received
        res.goal_achieved = self.goal_achieved
        res.goal_pose_in_map = self.goal_pose_in_map
        return res


if __name__ == "__main__":
    rospy.init_node("goal_receiver_node")
    print(colored("goal_receiver_node initialized", 'yellow'))
    goal_receiver = goal_receiver_node()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        goal_receiver.visualize()
        rate.sleep()