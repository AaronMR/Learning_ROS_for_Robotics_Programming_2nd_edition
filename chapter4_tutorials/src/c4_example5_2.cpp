#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Int16.h>
#include<iostream>

using namespace std;
ros::Publisher pub;
ros::Subscriber sub;

void rangeCallBack(const std_msgs::Int16 &range)
{	
	geometry_msgs::Twist vel;
	if (range.data > 40)
	{
		vel.angular.z = 0;
		vel.linear.x = 1;
	}
	else if (range.data >20)
	{
		vel.angular.z = 1;
		vel.linear.x = 0;

	}
	else
	{
		vel.angular.z = 0;
		vel.linear.x = -1;

	}
	pub.publish(vel);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtle_ultra_sound");
	ros::NodeHandle n;
	pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);
	sub = n.subscribe("range", 10, &rangeCallBack);
	ros::spin();
}
