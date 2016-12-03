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
    
    
    
    
    
