
#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "example9" );

  ros::NodeHandle n;

  ros::Publisher pub_marker = n.advertise< visualization_msgs::Marker >( "marker", 1000 );
  ros::Publisher pub_pc = n.advertise< sensor_msgs::PointCloud2 >( "pc", 1000 );

  visualization_msgs::Marker msg_marker;

  msg_marker.header.frame_id = "/frame_marker";

  msg_marker.ns = "shapes";
  msg_marker.id = 0;

  msg_marker.type = visualization_msgs::Marker::CUBE;

  msg_marker.action = visualization_msgs::Marker::ADD;

  msg_marker.pose.position.x = 0.;
  msg_marker.pose.position.y = 1.;
  msg_marker.pose.position.z = 2.;
  msg_marker.pose.orientation.x = 0.;
  msg_marker.pose.orientation.y = 0.;
  msg_marker.pose.orientation.z = 0.;
  msg_marker.pose.orientation.w = 1.;

  msg_marker.scale.x = 1.;
  msg_marker.scale.y = 1.;
  msg_marker.scale.z = 1.;

  msg_marker.color.r = 1.;
  msg_marker.color.g = 0.;
  msg_marker.color.b = 0.;
  msg_marker.color.a = 1.;

  msg_marker.lifetime = ros::Duration();

  ROS_INFO_STREAM( "Initial Marker created." );

  sensor_msgs::PointCloud2 msg_pc;

  pcl::PointCloud< pcl::PointXYZ > pc;

  pc.width  = 200;
  pc.height = 100;
  pc.is_dense = false;
  pc.points.resize( pc.width * pc.height );

  for( size_t i = 0; i < pc.height; ++i ) {
    for( size_t j = 0; j < pc.width; ++j ) {
      const size_t k = pc.width * i + j;

      pc.points[k].x = 0.1 * i;
      pc.points[k].y = 0.2 * j;
      pc.points[k].z = 1.5;
    }
  }

  ROS_INFO_STREAM( "Initial Point Cloud created." );

  ros::Rate rate( 1 );
  while( ros::ok() ) {
    msg_marker.header.stamp = ros::Time::now();

    msg_marker.pose.position.x += 0.01;
    msg_marker.pose.position.y += 0.02;
    msg_marker.pose.position.z += 0.03;

    for( size_t i = 0; i < pc.height; ++i ) {
      for( size_t j = 0; j < pc.width; ++j ) {
        const size_t k = pc.width * i + j;

        pc.points[k].z -= 0.1;
      }
    }

    pcl::toROSMsg( pc, msg_pc );

    msg_pc.header.stamp = msg_marker.header.stamp;
    msg_pc.header.frame_id = "/frame_pc";

    pub_marker.publish( msg_marker );
    pub_pc.publish( msg_pc );

    ros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}

