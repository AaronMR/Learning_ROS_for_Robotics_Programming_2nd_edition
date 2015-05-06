#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>

geometry_msgs::Point global_position;

ros::Publisher position_pub;
void gpsCallBack(const sensor_msgs::NavSatFixConstPtr& gps)
{
	double northing, easting;
  	char zone;
	//LLtoUTM(gps->latitude, gps->longitude,  northing, easting , &zone);
	global_position.x = easting;
	global_position.y = northing;
	global_position.z = gps->altitude;
}

int main(int argc, char** argv){
	ros::init(argc,argv, "Geoposition");
	ros::NodeHandle n;
	ros::Subscriber gps_sub = n.subscribe("fix",10, gpsCallBack);
	position_pub = n.advertise<geometry_msgs::Point> ("global_position", 1);
	ros::Rate loop_rate(10);
	while(n.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
