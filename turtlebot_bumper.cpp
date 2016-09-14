#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "im_msgs/BumperState.h"

void bumperCallback(const im_msgs::BumperState::ConstPtr& st)
{
  geometry_msgs::Twist vel;
  vel.linear.x = 0.1;
  if(st->bumper_state)
    {
      vel.angular.z = 0.5;
    }
    else
    {
        vel.angular.z = 0;
    }
  
}

int main(int argc, char **argv)
{

  //vel.linear.x = 0;//linear velocity(m/s)
  //vel.angular.z = 1.0;//angular velocity(rad/s)
  
  ros::init(argc, argv, "turtlebot_bumper");
  
  ros::NodeHandle nh;
  
  ros::Publisher vel_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1, true);
  
  ros::Subscriber sub = nh.subscribe<im_msgs::BumperState>("st", 10, bumperCallback);
  
  ros::Rate loop_rate(10);


  while (ros::ok())
  {
    vel_pub.publish(vel);
    ros::spinOnce();
    loop_rate.sleep();

  } 
  return 0;
}
