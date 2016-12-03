#!/usr/bin/env python

import rospy 
import numpy as np 
import math
import time
import tf
from std_msgs.msg import String 
from std_msgs.msg import Empty 
from kobuki_msgs.msg import BumperEvent 
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


	
#def bumperCallback(data):
#     if (data.state == BumperEvent.RELEASED ):
#          rospy.loginfo("no press!\n")
#     else:
#          rospy.loginfo("press!\n")

class BlockDef:
  def __init__(self, number, coordinate, neighbor, distance):
    self.number = number
    self.coordinate = coordinate
    self.neighbor = neighbor
    self.distance = distance
    self.isObstacle = False

def grassfire():
  B_1 = BlockDef(1, (0,0), (2,16), 100)
  B_2 = BlockDef(2, (1,0), (1,3), 100)
  B_3 = BlockDef(3, (2,0), (2,4), 100)
  B_4 = BlockDef(4, (3,0), (3,5), 100)
  B_5 = BlockDef(5, (4,0), (4,6), 100)
  B_6 = BlockDef(6, (4,1), (5,7), 100)
  B_7 = BlockDef(7, (4,2), (6,8), 100)
  B_8 = BlockDef(8, (4,3), (7,9), 100)
  B_9 = BlockDef(9, (4,4), (8,10), 100)
  B_10 = BlockDef(10, (3,4), (9,11), 100)
  B_11 = BlockDef(11, (2,4), (10,12), 100)
  B_12 = BlockDef(12, (1,4), (11,13), 100)
  B_13 = BlockDef(13, (0,4), (12,14), 100)
  B_14 = BlockDef(14, (0,3), (13,15), 100)
  B_15 = BlockDef(15, (0,2), (14,16), 100)
  B_16 = BlockDef(16, (0,1), (15,1), 100)

  BlockList = [B_1, B_2, B_3, B_4, B_5, B_6, B_7, B_8, B_9, B_10, B_11,
             B_12, B_13, B_14, B_15, B_16]

  Start = BlockList[0]
  Goal = BlockList[9]

  # B_14.isObstacle = True

  CalculateList = []
  Goal.distance = 0
  CalculateList.append(Goal)
  

  while len(CalculateList) != 0:
    current = CalculateList[0]
    CalculateList.remove(CalculateList[0])
    for num in current.neighbor:
      if BlockList[num - 1].distance == 100 and BlockList[num - 1].isObstacle == False:
        BlockList[num - 1].distance = current.distance + 1
        CalculateList.append(BlockList[num - 1])

  ResultList = []
  iterator = Start
  iteration = Start.distance

  while iteration != 0:
    ResultList.append(iterator)
    # tem = []
    for block in iterator.neighbor:
      if BlockList[block - 1].distance < iterator.distance:
        iterator = BlockList[block - 1]
    iteration = iteration - 1

  ResultList.append(Goal)
  return ResultList

 
class Odom():
  def __init__(self):
      self.posx = 0.0
      self.posy = 0.0
      self.vel = 0.0
      self.theta = 0.0
      rospy.Subscriber('odom',Odometry,self.odomCallback)
  def odomCallback(self,msg):
      self.posx = msg.pose.pose.position.x
      self.posy = msg.pose.pose.position.y
      self.theta = msg.pose.pose.orientation.z
      self.vel = msg.twist.twist.linear.x
 
class Laser():
  def __init__(self):
      self.closest = 0.0
      self.position = 0
      rospy.Subscriber("/scan",LaserScan,self.laserCallback,queue_size=1)

  def laserCallback(self,scan):
      depths = []
      for dist in scan.ranges:
          if not np.isnan(dist):
             depths.append(dist)
     
      fullDepthsArray = list(scan.ranges[:]) 
      if len(depths) == 0:
         self.closest = 0
         self.position = 0
      else:
         self.closest = min(depths)
    
class Bumper():
  press = False;
  def __init__(self):
     rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.bumperCallback)

  def bumperCallback(self,data):
     if (data.state == BumperEvent.RELEASED ):
          self.press = False;
     else:
          self.press = True;
 
  def shutdown(self):
       rospy.loginfo("Stopping the bumper sensor...")
       rospy.sleep(1)


def move(theta_now,xy_now,xy_next):
      dxy = xy_next - xy_now
      print "P2"
      theta = math.atan2(dxy[1],dxy[0])
      print "P3"
      dtheta = theta - theta_now
      dist = np.sqrt(dxy[0]**2 + dxy[1]**2)
      return([dtheta,dxy]) 


def obs_stop_py(result):
     Ka = 1.0
     Kv = 0.5
     vel_ref = 0.2
     rospy.init_node('obs_stop_py',anonymous=True)
     print "hello\n"
     b1 = Bumper() 
     O1 = Odom()
     velocity_publisher = rospy.Publisher("cmd_vel_mux/input/navi",Twist,queue_size=10)
     vel_msg = Twist()

     r = rospy.Rate(10)
     flag = 0
     starter = result[0]
     result.remove(result[0])
     theta_now = 0.0
     scale = 0.5
      
#     for step in range(30):
#           vel_msg.angular.z = 0.1
#           vel_msg.linear.x = 0.0
#           velocity_publisher.publish(vel_msg)
#           r.sleep()
     print "P0"
     pub = rospy.Publisher("/mobile_base/commands/reset_odometry", Emtpy, queue_size=10)
     pub.publish()
     print "P0"
     time.sleep(1) 
     print "initial position",O1.posx,",",O1.posy, "theta:", O1.theta
     while (len(result)>0): 
           dtheta, dxy = move(theta_now, scale * np.array(starter.coordinate), scale*np.array(result[0].coordinate))
           # implement go2goal behavior
           goalx = O1.posx + dxy[0] 
           goaly = O1.posy + dxy[1] 
           dist = np.sqrt( dxy[0]**2 + dxy[1]**2 )
           print "goal:",goalx,",",goaly
           while (not rospy.is_shutdown() and dist > 0.1):
               print "current position",O1.posx,",",O1.posy, "theta:", O1.theta
               print "distance:", dist 
               dist = np.sqrt( (goalx-O1.posx)**2 + (goaly-O1.posy)**2 )
               dtheta = math.atan2(goaly-O1.posy,goalx-O1.posx)
               vel_msg.angular.z = Ka * (math.fmod(dtheta - O1.theta * np.pi + np.pi, 2*np.pi) - np.pi)
               #vel_msg.linear.x = vel_ref + Kv * (vel_ref - O1.vel)
               vel_msg.linear.x = 0.05
              # print "angular, linear:",vel_msg.angular.z, vel_msg.linear.x
               velocity_publisher.publish(vel_msg)
               r.sleep()
           # turn to the correct angle first
#           vel_msg.angular.z = dtheta/10
#           vel_msg.linear.x = 0.0
#           for step in range(100):
#               velocity_publisher.publish(vel_msg)
#	       r.sleep()
#           vel_msg.angular.z = 0.0
#           vel_msg.linear.x = dist/10
#           for step in range(100):
#               velocity_publisher.publish(vel_msg)
#	       r.sleep()
           
           theta_now = theta_now + dtheta 
           starter = result[0]
           result.remove(result[0])

#     rospy.spin()
     


if __name__ == '__main__':
 #       print "hello1\n"
   try:
     result = grassfire()
     obs_stop_py(result)
   except:
     rospy.loginfo("node terminated.")
#     pub = rospy.Pub
