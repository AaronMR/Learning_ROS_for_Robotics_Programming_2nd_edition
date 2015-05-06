// Example based on interactive_marker_tutorials (simple_marker)

#include <ros/ros.h>
#include <tf/tf.h>

#include <interactive_markers/interactive_marker_server.h>

void feedback_callback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  double roll, pitch, yaw;
  tf::Quaternion q;
  tf::quaternionMsgToTF(feedback->pose.orientation, q);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  ROS_INFO_STREAM(
    feedback->marker_name << " position (x, y, z) = (" <<
    feedback->pose.position.x << ", " <<
    feedback->pose.position.y << ", " <<
    feedback->pose.position.z << "), orientation (roll, pitch, yaw) = (" <<
    roll << ", " << pitch << ", " << yaw << ")"
  );
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "example10");

  interactive_markers::InteractiveMarkerServer server("marker");

  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = "/base_link";
  marker.name = "marker";
  marker.description = "2-DOF Control";

  // Box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.5;
  box_marker.scale.y = 0.5;
  box_marker.scale.z = 0.5;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // Non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);

  // Controls to move the box
  visualization_msgs::InteractiveMarkerControl move_x_control, rotate_z_control;
  move_x_control.name = "move_x";
  move_x_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  rotate_z_control.name = "rotate_z";
  rotate_z_control.orientation.w = 1;
  rotate_z_control.orientation.y = 1;
  rotate_z_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

  marker.controls.push_back(box_control);
  marker.controls.push_back(move_x_control);
  marker.controls.push_back(rotate_z_control);

  server.insert(marker, &feedback_callback);
  server.applyChanges();

  ros::spin();
}

