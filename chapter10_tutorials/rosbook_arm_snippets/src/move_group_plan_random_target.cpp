#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit/robot_state/conversions.h>


int main(int argc, char **argv)
{
    // Initialize ROS, create the node handle and an async spinner
    ros::init(argc, argv, "move_group_plan_random_target");
    ros::NodeHandle nh;

    ros::AsyncSpinner spin(1);
    spin.start();

    // Get the arm planning group
    moveit::planning_interface::MoveGroup plan_group("arm");

    // Create a published for the arm plan visualization
    ros::Publisher display_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    // Service client for checking state validity validity_srv
    ros::ServiceClient validity_srv =  nh.serviceClient<moveit_msgs::GetStateValidity>("/check_state_validity");

    // Get the current RobotState, which will be used to set the arm
    // to a random state
    robot_state::RobotState current_state = *plan_group.getCurrentState();

    // Generate a new random state and put it into the request
    current_state.setToRandomPositions(current_state.getJointModelGroup("arm"));

    moveit_msgs::GetStateValidity::Request validity_request;
    moveit_msgs::GetStateValidity::Response validity_response;

    robot_state::robotStateToRobotStateMsg(current_state, validity_request.robot_state);
    validity_request.group_name = "arm";

    // Perform the service call to verify the validity of the state, if it fails
    // we abort the operation
    validity_srv.call(validity_request, validity_response);

    if (!validity_response.valid)
    {
        ROS_INFO("Random state is not valid");
        ros::shutdown();
        return 1;
    }

    // If we reached this point, the random goal is valid, so we set the target
    // values to the random state generated previously
    plan_group.setJointValueTarget(current_state);

    // Perform the planning step, and if it succeeds display the current
    // arm trajectory and move the arm
    moveit::planning_interface::MoveGroup::Plan goal_plan;
    if (plan_group.plan(goal_plan))
    {
        moveit_msgs::DisplayTrajectory display_msg;
        display_msg.trajectory_start = goal_plan.start_state_;
        display_msg.trajectory.push_back(goal_plan.trajectory_);
        display_pub.publish(display_msg);

        sleep(5.0);

        plan_group.move();
    }

    ros::shutdown();

    return 0;
}
