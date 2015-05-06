#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

using namespace std;

double width_robot = 0.1;
double vl = 0.0;
double vr = 0.0;
ros::Time last_time;	
double right_enc = 0.0;
double left_enc = 0.0;
double right_enc_old = 0.0;
double left_enc_old = 0.0;
double distance_left = 0.0;
double distance_right = 0.0;
double ticks_per_meter = 100;
double x = 0.0;
double y = 0.0;
double th = 0.0;
geometry_msgs::Quaternion odom_quat;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	geometry_msgs::Twist twist = twist_aux;	
	double vel_x = twist_aux.linear.x;
	double vel_th = twist_aux.angular.z;
	double right_vel = 0.0;
	double left_vel = 0.0;

	if(vel_x == 0){  // turning
		right_vel = vel_th * width_robot / 2.0;
		left_vel = (-1) * right_vel;
	}else if(vel_th == 0){ // fordward / backward
		left_vel = right_vel = vel_x;
	}else{ // moving doing arcs
		left_vel = vel_x - vel_th * width_robot / 2.0;
		right_vel = vel_x + vel_th * width_robot / 2.0;
	}
	vl = left_vel;
	vr = right_vel;	
}


int main(int argc, char** argv){
	ros::init(argc, argv, "base_controller");
	ros::NodeHandle n;
	ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 10, cmd_velCallback);
	ros::Rate loop_rate(10);

	while(ros::ok())
	{

		double dxy = 0.0;
		double dth = 0.0;
		ros::Time current_time = ros::Time::now();
		double dt;
		double velxy = dxy / dt;
		double velth = dth / dt;

		ros::spinOnce();
		dt =  (current_time - last_time).toSec();;
		last_time = current_time;

		// calculate odomety
		if(right_enc == 0.0){
			distance_left = 0.0;
			distance_right = 0.0;
		}else{
			distance_left = (left_enc - left_enc_old) / ticks_per_meter;
			distance_right = (right_enc - right_enc_old) / ticks_per_meter;
		} 

		left_enc_old = left_enc;
		right_enc_old = right_enc;

		dxy = (distance_left + distance_right) / 2.0;
		dth = (distance_right - distance_left) / width_robot;

		if(dxy != 0){
			x += dxy * cosf(dth);
			y += dxy * sinf(dth);
		}	

		if(dth != 0){
			th += dth;
		}
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);
		loop_rate.sleep();
	}
}
