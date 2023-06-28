#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys, copy
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from controller_window import Ui_Form
# ROS
import rospy
from geometry_msgs.msg import Pose
import moveit_commander

rospy.init_node('controller')

arm = moveit_commander.MoveGroupCommander("manipulator")


waypoints = []
arm_current_pose = Pose()
arm_current_pose = arm.get_current_pose()
arm.clear_pose_targets()
arm.set_goal_tolerance(0.0001)


target_pose = Pose()
target_pose.position = arm_current_pose.pose.position
target_pose.position.x = 0.1902998179030183
target_pose.position.y = 0.2597139008173162
target_pose.position.z = 0
target_pose.orientation = arm_current_pose.pose.orientation
target_pose.orientation.x = -0.7071068
target_pose.orientation.y = -0.7071068
target_pose.orientation.z = 0
target_pose.orientation.w =0

waypoints.append(copy.deepcopy(target_pose))
# (plan_success, plan, planning_time, error_code)=arm.plan()

print("target pose")
print(target_pose)
print("plan success")

(plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step

arm.execute(plan, wait=True)