#!/usr/bin/env python
import rospy

from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class RoboServer():
  def __init__(self):
    rospy.init_node('final', anonymous=True)
    
    # nodes:
    #======
    #                                         go to goal
    #                                             ^
    #                                             ||
    #                                             sub
    #                                             ||
    #  LaserScan == sub ==> LaserScan == pub ==> MAP <== pub == robot finder <== sub == Odometry
    #                      
    
    
    
    self.__cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
    self.__bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.__bumper_handler)
    self.__odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_handler)
    self.__scan_sub = rospy.Subscriber('/scan', LaserScan, self.__scan_handler)
    
    
