
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#include <chapter3_tutorials/SetSpeed.h>

float previous_speed = 0.;
float new_speed      = 0.;

void callback_temp( const std_msgs::Int32::ConstPtr& msg )
{
  ROS_INFO_STREAM( "Temp = " << msg->data );
}

void callback_accel( const geometry_msgs::Vector3::ConstPtr& msg )
{
  ROS_INFO_STREAM(
    "Accel = (" << msg->x << ", " << msg->y << ", " << msg->z << ")"
  );
}

bool callback_speed(
  chapter3_tutorials::SetSpeed::Request  &req,
  chapter3_tutorials::SetSpeed::Response &res
)
{
  ROS_INFO_STREAM(
    "speed service request: desired speed = " << req.desired_speed  
  );

  new_speed = 0.9 * req.desired_speed;

  res.previous_speed = previous_speed;
  res.new_speed      = new_speed;
  res.stalled        = new_speed < 0.1;

  previous_speed = new_speed;

  return true;
}


int main( int argc, char **argv )
{

  ros::init( argc, argv, "example5" );

  ros::NodeHandle n;

  ros::Subscriber sub_temp = n.subscribe( "temp", 1000, callback_temp );
  ros::Subscriber sub_accel = n.subscribe( "accel", 1000, callback_accel );

  ros::ServiceServer srv_speed = n.advertiseService( "speed", callback_speed );

  while( ros::ok() ) {
    ros::spin();
  }

  return EXIT_SUCCESS;

}

