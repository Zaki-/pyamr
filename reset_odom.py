#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Empty
#sys.path.append("/home/amr")

#import odom_reset

from nav_msgs.msg import Odometry
def reset():
    pub = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

#    for i in range(0,10):
    pub.publish()
def odometryCb(msg):
    m = msg.pose.pose.position.x
    print m
    if (m > 2):
        reset()

if __name__ == "__main__":
    rospy.init_node('reset_odom', anonymous=True)  
    rospy.Subscriber('odom',Odometry,odometryCb)

    rospy.spin()
