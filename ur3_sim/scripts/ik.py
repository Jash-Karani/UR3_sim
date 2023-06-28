#! /usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK

rospy.init_node("controller")

# Create a service proxy for the inverse kinematics service
ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)

# Create a request for the inverse kinematics
ik_request = PositionIKRequest()

# Set the group name for the arm you want to control
ik_request.group_name = "manipulator"

# Set the pose of the end-effector for which you want to compute IK
ik_request.pose_stamped.header.frame_id = "world"
ik_request.pose_stamped.pose.position.x = 0.1902998179030183
ik_request.pose_stamped.pose.position.y = 0.2597139008173162
ik_request.pose_stamped.pose.position.z = 0
ik_request.pose_stamped.pose.orientation.x = -0.7071068
ik_request.pose_stamped.pose.orientation.y =-0.7071068
ik_request.pose_stamped.pose.orientation.z = 0.0
ik_request.pose_stamped.pose.orientation.w = 0

# Set the timeout for the IK computation
ik_request.timeout = rospy.Duration(5.0)

# Call the IK service to compute the IK solution
ik_response = ik_service(ik_request)

# Check if IK computation was successful
if ik_response.error_code.val == ik_response.error_code.SUCCESS:
    # Get the joint angles for the IK solution
    joint_angles = ik_response.solution.joint_state.position
    print("IK solution found!")
    print("Joint angles: ", joint_angles)
else:
    print("IK computation failed with error code: ", ik_response.error_code.val)
