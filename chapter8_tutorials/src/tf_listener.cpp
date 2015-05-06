#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame

  geometry_msgs::PointStamped laser_point;
  laser_point.header.frame_id = "base_laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 2.0;
  laser_point.point.z = 0.0;

  geometry_msgs::PointStamped base_point;
 

  //listener.waitForTransform(

  listener.transformPoint("base_link", laser_point, base_point);


//  listener.lookupTransform("base_link", "wheel_2", ros::Time(0), ros::Duration(10.0));
  ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());

}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
