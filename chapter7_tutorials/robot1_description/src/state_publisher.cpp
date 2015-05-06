#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double inc= 0.005, base_arm_inc= 0.005, arm1_armbase_inc= 0.005, arm2_arm1_inc= 0.005, gripper_inc= 0.005, tip_inc= 0.005;
    double angle= 0 ,base_arm = 0, arm1_armbase = 0, arm2_arm1 = 0, gripper = 0, tip = 0;
    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(11);
        joint_state.position.resize(11);
        joint_state.name[0] ="base_to_arm_base";
        joint_state.position[0] = base_arm;
        joint_state.name[1] ="arm_1_to_arm_base";
        joint_state.position[1] = arm1_armbase;
        joint_state.name[2] ="arm_2_to_arm_1";
        joint_state.position[2] = arm2_arm1;
	joint_state.name[3] ="left_gripper_joint";
        joint_state.position[3] = gripper;
	joint_state.name[4] ="left_tip_joint";
        joint_state.position[4] = tip;
	joint_state.name[5] ="right_gripper_joint";
        joint_state.position[5] = gripper;
	joint_state.name[6] ="right_tip_joint";
        joint_state.position[6] = tip;
	joint_state.name[7] ="base_to_wheel1";
        joint_state.position[7] = 0;
	joint_state.name[8] ="base_to_wheel2";
        joint_state.position[8] = 0;
	joint_state.name[9] ="base_to_wheel3";
        joint_state.position[9] = 0;
	joint_state.name[10] ="base_to_wheel4";
        joint_state.position[10] = 0;






        // update transform
        // (moving in a circle with radius)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle);
        odom_trans.transform.translation.y = sin(angle);
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);


	// Create new robot state
        arm2_arm1 += arm2_arm1_inc;
        if (arm2_arm1<-1.5 || arm2_arm1>1.5) arm2_arm1_inc *= -1;
		  arm1_armbase += arm1_armbase_inc;
        if (arm1_armbase>1.2 || arm1_armbase<-1.0) arm1_armbase_inc *= -1;
        base_arm += base_arm_inc;
        if (base_arm>1. || base_arm<-1.0) base_arm_inc *= -1;
        gripper += gripper_inc;
        if (gripper<0 || gripper>1) gripper_inc *= -1;
		
		  angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
