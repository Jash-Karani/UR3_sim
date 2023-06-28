#!/usr/bin/env python3
import roslib
# roslib.load_manifest('learning_tf')
import rospy

import tf
from fiducial_msgs.msg import dictmsg
from geometry_msgs.msg import Transform,Vector3,Quaternion
import time

if __name__ == '__main__':
    from std_msgs.msg import String
    pub = rospy.Publisher('/aruco_pose', dictmsg, queue_size=10)
    rospy.init_node('aruco_pose')

    listener = tf.TransformListener()

    final=dictmsg()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        for i in range(1,15):
            if i not in final.keys:
                try:
                    
                    (trans,rot) = listener.lookupTransform('/world', '/fiducial_'+str(i), rospy.Time(0))
                    final.keys.append(i)
                    final.values.append(Transform(Vector3(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3])))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    print("failed for ",i)
                    continue


        print(final.keys)
     
        pub.publish(final)

        rate.sleep()