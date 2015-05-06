
#include <ros/ros.h>
#include <ros/console.h>

#define OVERRIDE_NODE_VERBOSITY_LEVEL 1

int main( int argc, char **argv )
{

  ros::init( argc, argv, "example1" );

#if OVERRIDE_NODE_VERBOSITY_LEVEL
  // Se the logging level manually to DEBUG
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
#endif

  ros::NodeHandle n;

  const double val = 3.14;

  ROS_DEBUG( "This is a simple DEBUG message!" );

  ROS_DEBUG( "This is a DEBUG message with an argument: %f", val );

  ROS_DEBUG_STREAM(
    "This is DEBUG stream message with an argument: " << val
  );

  ros::spinOnce();

  return EXIT_SUCCESS;

}

