#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list




moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)
planning_frame = move_group.get_planning_frame()
eef_link = move_group.get_end_effector_link()
group_names = robot.get_group_names()
# print(robot.get_current_state())
# print("")


pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 0.7
pose_goal.orientation.y = 0.7
pose_goal.orientation.x = 0.7
pose_goal.orientation.z = 0.0
pose_goal.position.x = 0.28
pose_goal.position.y = 0.11
pose_goal.position.z = 0.35
# print(pose_goal)
move_group.set_pose_target(pose_goal)
# `go()` returns a boolean indicating whether the planning and execution was successful.
# success = move_group.go(wait=True)
plan_success, plan, planning_time, error_code=move_group.plan()
print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
print(plan)
print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
move_group.execute(plan, wait=True)
# print(robot.get_current_state())
# Calling `stop()` ensures that there is no residual movement
# move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets().
moveit_commander.roscpp_shutdown()
