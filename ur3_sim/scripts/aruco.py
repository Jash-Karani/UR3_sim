#!/usr/bin/env python3
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
import time

if __name__ == '__main__':
    from std_msgs.msg import String
    # pub = rospy.Publisher('/aruco_pose', String, queue_size=10)
    rospy.init_node('aruco_pose')

    listener = tf.TransformListener()

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    # turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # (trans_10,rot10) = listener.lookupTransform('/world', '/fiducial_10', rospy.Time(0))
            # (trans_10,rot10) = listener.lookupTransform('/world', '/fiducial_10', rospy.Time(0))
            (trans_11,rot11) = listener.lookupTransform('/world', '/fiducial_11', rospy.Time(0))
            (trans_11,rot11) = listener.lookupTransform('/world', '/fiducial_11', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("failed")
            continue

        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)
        # print("aruco 10: ",trans_10,rot10)
        print("aruco 11: ",trans_11,rot11)
        # dict={}
        # pub.publish(dict)

        rate.sleep()