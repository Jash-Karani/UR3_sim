#! /usr/bin/python3
# -*- coding: utf-8 -*-
import time
import sys, copy
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *

# ROS
import rospy
from geometry_msgs.msg import Pose
import moveit_commander
from std_msgs.msg import String


my_param_value = rospy.get_param('test')

# Use the parameter
print("The value of 'test' is:", my_param_value)

def gotoxyz(x,y,z):
    waypoints = []
    arm_current_pose = Pose()
    arm_current_pose = arm.get_current_pose()
    arm.clear_pose_targets()
    arm.set_goal_tolerance(0.01)


    target_pose = Pose()
    # target_pose.position = arm_current_pose.pose.position
    # target_pose.position.x = x-0.097
    # target_pose.position.y = y
    # target_pose.position.z = z-0.055
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    # target_pose.orientation = arm_current_pose.pose.orientation
    target_pose.orientation.x = -1/(2**(1/2))
    target_pose.orientation.y = -1/(2**(1/2))
    target_pose.orientation.z = 0
    target_pose.orientation.w = 0 

    waypoints.append(copy.deepcopy(target_pose))

    print("target pose")
    print(target_pose)
    arm.set_pose_target(target_pose)
    plan_success,plan,planning_time,error_code=arm.plan()
    arm.execute(plan)
    # (plan, fraction) = arm.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow  # eef_step
    # arm.execute(plan, wait=True)
    

rospy.init_node('controller')


arm = moveit_commander.MoveGroupCommander("manipulator")

x=0.20
y=0.28
z=-0.046
#gotoxyz(x-0.1,y,z)
#time.sleep(5)
gotoxyz(x,y,z)