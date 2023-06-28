#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import moveit_commander
import math
import tf
from fiducial_msgs.msg import dictmsg
import time
import sys, copy
from geometry_msgs.msg import Transform,Vector3,Quaternion
import time
import subprocess

counter=0
arm = moveit_commander.MoveGroupCommander("manipulator")
def move_imu(pose,orientaion):
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
    target_pose.position.x = pose[0]
    target_pose.position.y = pose[1]
    target_pose.position.z = pose[2]
    # target_pose.orientation = arm_current_pose.pose.orientation
    target_pose.orientation.x = orientaion[0]
    target_pose.orientation.y = orientaion[1]
    target_pose.orientation.z = orientaion[2]
    target_pose.orientation.w = orientaion[3]

    waypoints.append(copy.deepcopy(target_pose))

    print("target pose")
    print(target_pose)
    arm.set_pose_target(target_pose)
    plan_success,plan,planning_time,error_code=arm.plan()
    arm.execute(plan)


class quaternion:
    def __init__(self,x,y,z,c):
        d = math.sqrt(x**2 + y**2 + z**2 + c**2)
        self.x = x/d
        self.y = y/d
        self.z = z/d
        self.c = c/d
    def invq(self): #inverse quaternion function
        return quaternion(-1*(self.x),-1*(self.y),-1*(self.z),self.c)
    def rotq(self,q2 : "quaternion"): # quaternion rotation
        r = quaternion(self.x,self.y,self.z,self.c)
        r.x = self.c*q2.x +self.x*q2.c -self.y*q2.z +self.z *q2.y
        r.y = self.c*q2.y + self.x*q2.z +self.y*q2.c - self.z*q2.x
        r.z = self.c*q2.z - self.x*q2.y + self.y*q2.x +self.z*q2.c
        r.c = self.c*q2.c -(self.x*q2.x + self.y*q2.y + self.z*q2.z)
        return r
    def neg(self):
        return quaternion(-1*(self.x),-1*(self.y),-1*(self.z),-1*(self.c))

def get_right_orientation(aruco_10_orien_temp):
    #intial values
    a=aruco_10_orien_temp[0]
    b=aruco_10_orien_temp[1]
    c=aruco_10_orien_temp[2]
    d=aruco_10_orien_temp[3]

    imu_q_i = quaternion(0.0032026,0.00351542,0.7084,0.70579)
    tool_q_i = quaternion(0.70506,0.70914,0.002042,0.0023077)
    imuTotool = ((imu_q_i.invq()).rotq(tool_q_i))
    

    #input current imu parameters
    currentIMU = quaternion(a,b,c,d) 
    toolQuat = currentIMU.rotq(imuTotool) #the required quternion for the tool 
    print("**",[toolQuat.x,toolQuat.y,toolQuat.z,toolQuat.c],"**")
    return [toolQuat.x,toolQuat.y,toolQuat.z,toolQuat.c]


def grip_imu(aruco_10_pos,aruco_10_orien):
    aruco_10_pos[2]=aruco_10_pos[2]+0.040
    gripper_orientation=get_right_orientation(aruco_10_orien)
    move_imu(aruco_10_pos,gripper_orientation)
    

    pub = rospy.Publisher('/gripper_command', String, queue_size=10)
    rospy.sleep(4)

    print(pub)
    command="semi_open"
    rospy.loginfo(command)
    rospy.sleep(4)

    pub.publish(command)
    rospy.sleep(4)

    
def stick_position(aruco_11_pos):
    pass



def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print("2")
    aruco_11 = data.keys.index(11)
    aruco_10 = data.keys.index(10)
    aruco_11_data=(data.values[aruco_11])
    aruco_10_data=(data.values[aruco_10])
    aruco_10_pos=[aruco_10_data.translation.x,aruco_10_data.translation.y,aruco_10_data.translation.z]
    aruco_10_orien=[aruco_10_data.rotation.x,aruco_10_data.rotation.y,aruco_10_data.rotation.z,aruco_10_data.rotation.w]
    aruco_11_pos=[aruco_11_data.translation.x,aruco_11_data.translation.y,aruco_11_data.translation.z]
    aruco_11_orien=[aruco_11_data.rotation.x,aruco_11_data.rotation.y,aruco_11_data.rotation.z,aruco_11_data.rotation.w]
    
    grip_imu(aruco_10_pos,aruco_10_orien)
    # final_gripper_orientation

# def listener():
rospy.init_node('imu_aruco_mover', anonymous=True)
rate = rospy.Rate(1)
# rospy.Subscriber("/aruco_pose", dictmsg, callback)
data = rospy.wait_for_message('/aruco_pose', dictmsg, timeout=5)
callback(data)
print("1")
# rospy.sleep(1)
# Call spinOnce() to process a single message
# rospy.spin()


# if __name__ == '__main__':
#     try:
#         listener()
#     except rospy.ROSInterruptException:
#         pass