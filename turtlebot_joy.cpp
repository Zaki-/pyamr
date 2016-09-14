#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;

  if(joy->buttons[4])
    {
    vel.linear.x = 0.1;
    }
    else
    {
        vel.linear.x = 0;
    }
  if(joy->buttons[3])
    {
    vel.angular.z = 0.1;
    }
    else
    {
        vel.linear.x = 0;
    }
}

int main(int argc, char **argv)
{

  //vel.linear.x = 0;//linear velocity(m/s)
  //vel.angular.z = 1.0;//angular velocity(rad/s)
  
  ros::init(argc, argv, "turtlebot_joy");
  
  ros::NodeHandle nh;
  
  ros::Publisher vel_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1, true);
  
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
  
  ros::Rate loop_rate(10);


  while (ros::ok())
  {
    vel_pub.publish(vel);
    ros::spinOnce();
    loop_rate.sleep();

  } 
  return 0;
}
