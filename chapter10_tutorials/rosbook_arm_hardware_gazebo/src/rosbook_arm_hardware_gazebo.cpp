
#include <angles/angles.h>

#include <urdf_parser/urdf_parser.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <pluginlib/class_list_macros.h>

#include <rosbook_arm_hardware_gazebo/rosbook_arm_hardware_gazebo.h>

namespace rosbook_arm_hardware_gazebo
{
  using namespace hardware_interface;

  ROSBookArmHardwareGazebo::ROSBookArmHardwareGazebo()
    : gazebo_ros_control::RobotHWSim()
  {}


  bool ROSBookArmHardwareGazebo::initSim(const std::string& robot_namespace,
      ros::NodeHandle nh,
      gazebo::physics::ModelPtr model,
      const urdf::Model* const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    using gazebo::physics::JointPtr;

    // Cleanup
    sim_joints_.clear();
    jnt_pos_.clear();
    jnt_vel_.clear();
    jnt_eff_.clear();
    jnt_pos_cmd_.clear();

    // Simulation joints
    sim_joints_ = model->GetJoints();
    n_dof_ = sim_joints_.size();

    std::vector<std::string> jnt_names;
    for (size_t i = 0; i < n_dof_; ++i)
    {
      jnt_names.push_back(sim_joints_[i]->GetName());
    }

    // Raw data
    jnt_pos_.resize(n_dof_);
    jnt_vel_.resize(n_dof_);
    jnt_eff_.resize(n_dof_);
    jnt_pos_cmd_.resize(n_dof_);

    // Hardware interfaces
    for (size_t i = 0; i < n_dof_; ++i)
    {
      jnt_state_interface_.registerHandle(
          JointStateHandle(jnt_names[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]));

      jnt_pos_cmd_interface_.registerHandle(
          JointHandle(jnt_state_interface_.getHandle(jnt_names[i]), &jnt_pos_cmd_[i]));

      ROS_DEBUG_STREAM("Registered joint '" << jnt_names[i] << "' in the PositionJointInterface.");
    }

    registerInterface(&jnt_state_interface_);
    registerInterface(&jnt_pos_cmd_interface_);

    // Position joint limits interface
    std::vector<std::string> cmd_handle_names = jnt_pos_cmd_interface_.getNames();
    for (size_t i = 0; i < n_dof_; ++i)
    {
      const std::string name = cmd_handle_names[i];
      JointHandle cmd_handle = jnt_pos_cmd_interface_.getHandle(name);

      using namespace joint_limits_interface;
      boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(name);
      JointLimits limits;
      SoftJointLimits soft_limits;
      if (!getJointLimits(urdf_joint, limits) || !getSoftJointLimits(urdf_joint, soft_limits))
      {
        ROS_WARN_STREAM("Joint limits won't be enforced for joint '" << name << "'.");
      }
      else
      {
        jnt_limits_interface_.registerHandle(
            PositionJointSoftLimitsHandle(cmd_handle, limits, soft_limits));

        ROS_DEBUG_STREAM("Joint limits will be enforced for joint '" << name << "'.");
      }
    }

    // PID controllers
    pids_.resize(n_dof_);
    for (size_t i = 0; i < n_dof_; ++i)
    {
      ros::NodeHandle joint_nh(nh, "gains/" + jnt_names[i]);

      if (!pids_[i].init(joint_nh))
      {
        return false;
      }
    }

    return true;
  }

  void ROSBookArmHardwareGazebo::readSim(ros::Time time, ros::Duration period)
  {
    for (size_t i = 0; i < n_dof_; ++i)
    {
      jnt_pos_[i] += angles::shortest_angular_distance
          (jnt_pos_[i], sim_joints_[i]->GetAngle(0u).Radian());
      jnt_vel_[i] = sim_joints_[i]->GetVelocity(0u);
      jnt_eff_[i] = sim_joints_[i]->GetForce(0u);
    }
  }

  void ROSBookArmHardwareGazebo::writeSim(ros::Time time, ros::Duration period)
  {
    // Enforce joint limits
    jnt_limits_interface_.enforceLimits(period);

    // Compute and send commands
    for (size_t i = 0; i < n_dof_; ++i)
    {
      const double error = jnt_pos_cmd_[i] - jnt_pos_[i];
      const double effort = pids_[i].computeCommand(error, period);

      sim_joints_[i]->SetForce(0u, effort);
    }
  }

} // namespace rosbook_hardware_gazebo

PLUGINLIB_EXPORT_CLASS(rosbook_arm_hardware_gazebo::ROSBookArmHardwareGazebo, gazebo_ros_control::RobotHWSim)
