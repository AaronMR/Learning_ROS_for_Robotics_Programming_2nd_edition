
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>

#include <chapter3_tutorials/SetSpeed.h>

int main( int argc, char **argv )
{

  ros::init( argc, argv, "example4" );

  ros::NodeHandle n;

  ros::Publisher pub_temp = n.advertise< std_msgs::Int32 >( "temp", 1000 );
  ros::Publisher pub_accel = n.advertise< geometry_msgs::Vector3 >( "accel", 1000 );

  ros::ServiceClient srv_speed = n.serviceClient< chapter3_tutorials::SetSpeed>( "speed" );

  std_msgs::Int32 msg_temp;
  geometry_msgs::Vector3 msg_accel;
  chapter3_tutorials::SetSpeed msg_speed;

  int i = 0;

  ros::Rate rate( 1 );
  while( ros::ok() ) {

    msg_temp.data = i;

    msg_accel.x = 0.1 * i;
    msg_accel.y = 0.2 * i;
    msg_accel.z = 0.3 * i;

    msg_speed.request.desired_speed = 0.01 * i;

    pub_temp.publish( msg_temp );
    pub_accel.publish( msg_accel );

    if( srv_speed.call( msg_speed ) )
    {
      ROS_INFO_STREAM(
        "SetSpeed response:\n" <<
        "previous speed = " << msg_speed.response.previous_speed << "\n" <<
        "new      speed = " << msg_speed.response.new_speed      << "\n" <<
        "motor stalled  = " << ( msg_speed.response.stalled ? "true" : "false" )
      );
    }
    else
    {
      // Note that this might happen at the beginning, because
      // the service server could have not started yet!
      ROS_ERROR_STREAM( "Call to speed service failed!" );
    }

    ++i;
  
    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;

}

