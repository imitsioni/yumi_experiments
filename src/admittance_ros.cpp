#include <yumi_experiments/admittance_ros.hpp>
#include <pluginlib/class_list_macros.h>


  AdmittanceRos::AdmittanceRos() : controller_interface::MultiInterfaceController <JointVelocityInterface,
                                                                                   ForceTorqueSensorInterface> ()
  {
    nh_ = ros::NodeHandle("~");

    // if (!init(hardware_interface::RobotHW *hw, ros::NodeHandle &n))
    // {
    //   throw std::logic_error("Missing parameters for the admittance controller");
    // }
  }

  AdmittanceRos::~AdmittanceRos(){}

  // bool AdmittanceRos::init(hardware_interface::RobotHW *hw, ros::NodeHandle &n)
  bool AdmittanceRos::init()

  {
    std::string base_frame;
    double settling_time, damping_ratio, frequency;
    generic_control_toolbox::MatrixParser matrix_parser;

    if (!nh_.getParam("kinematic_chain_base_link", base_frame))
    {
      ROS_ERROR("Missing kinematic_chain_base_link parameter");
      return false;
    }

    if (!nh_.getParam("admittance/settling_time", settling_time))
    {
      ROS_ERROR("Missing admittance/settling_time parameter");
      return false;
    }

    if (!nh_.getParam("admittance/damping_ratio", damping_ratio))
    {
      ROS_ERROR("Missing admittance/damping_ratio parameter");
      return false;
    }

    if (!nh_.getParam("admittance/torque_dead_zone", torque_dead_zone_))
    {
      ROS_ERROR("Missing admittance/torque_dead_zone parameter");
      return false;
    }

    if (!nh_.getParam("admittance/force_dead_zone", force_dead_zone_))
    {
      ROS_ERROR("Missing admittance/force_dead_zone parameter");
      return false;
    }

    if (!nh_.getParam("admittance/position_offset", pos_offset_))
    {
      ROS_ERROR("Missing admittance/position_offset parameter");
      return false;
    }

    if (!nh_.getParam("admittance/max_linear_acceleration", max_lin_acc_))
    {
      ROS_ERROR("Missing admittance/max_linear_acceleration parameter");
      return false;
    }

    if (!nh_.getParam("admittance/max_angular_acceleration", max_ang_acc_))
    {
      ROS_ERROR("Missing admittance/max_angular_acceleration parameter");
      return false;
    }

    if (!nh_.getParam("admittance/max_linear_velocity", max_lin_vel_))
    {
      ROS_ERROR("Missing admittance/max_linear_velocity parameter");
      return false;
    }

    if (!nh_.getParam("admittance/max_angular_velocity", max_ang_vel_))
    {
      ROS_ERROR("Missing admittance/max_angular_velocity parameter");
      return false;
    }

    if (!nh_.getParam("admittance/angular_threshold", angular_threshold_))
    {
      ROS_ERROR("Missing admittance/angular_threshold parameter");
      return false;
    }

    if (!nh_.getParam("admittance/linear_threshold", linear_threshold_))
    {
      ROS_ERROR("Missing admittance/linear_threshold parameter");
      return false;
    }

    if (!nh_.getParam("admittance/avoid_joint_limits", avoid_joint_limits_))
    {
      ROS_ERROR("Missing admittance/avoid_joint_limits parameter");
      return false;
    }

    if(!matrix_parser.parseMatrixData(K_p_, "admittance/K_p", nh_))
    {
      return false;
    }

    if (K_p_.rows() != 6 || K_p_.cols() != 6)
    {
      ROS_ERROR("Invalid K_p dimensions. Must be 6x6");
      return false;
    }

    frequency = 4/(settling_time*damping_ratio);
    B_ = K_p_/(frequency*frequency);
    K_d_ = 2*damping_ratio*K_p_/frequency;

    kdl_manager_ = std::make_shared<generic_control_toolbox::KDLManager>(base_frame);

    std::vector<std::string> arm_names;
    arm_names.push_back("left_arm");
    arm_names.push_back("right_arm");

    for (int i = 0; i < 2; i++)
    {
      generic_control_toolbox::ArmInfo info;

      if(!generic_control_toolbox::getArmInfo(arm_names[i], info))
      {
        return false;
      }

      if (!info.has_ft_sensor)
      {
        ROS_ERROR("Arm %s require a ft sensor", arm_names[i].c_str());
        return false;
      }

      if(!generic_control_toolbox::setKDLManager(info, kdl_manager_))
      {
        return false;
      }

      if(!generic_control_toolbox::setWrenchManager(info, wrench_manager_))
      {
        return false;
      }

      eef_name_.push_back(info.kdl_eef_frame);
    }

    desired_pose_.resize(2);
    cart_vel_.resize(2);

    return true;
  }
  // void AdmittanceRos::update(const ros::Time &time, const ros::Duration &period)
  void AdmittanceRos::update()

  {
    // // TODO make it less stupid
    joint_command_pub_ = nh_.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_1_l/command", 10);
    std_msgs::Float64 random_shit;
    random_shit.data = 0.2;
    joint_command_pub_.publish(random_shit);
    std::cout << "AM I DOING IT? \n";

  }
  //
  //
  // bool AdmittanceRos::parseGoal(); // TODO maybe get the trajectory points here
  // void AdmittanceRos::computeAdmittanceError(const KDL::Frame &desired_pose, const KDL::Frame &pose, Vector6d &error) const;
  // Vector6d AdmittanceRos::computeCartesianAccelerations(const Vector6d &vel_eig, const KDL::Frame &pose, const Eigen::Affine3d &desired_pose, const Vector6d &wrench) const;
  // Vector6d AdmittanceRos::applyDeadZone(const Vector6d &wrench) const;
  // Vector6d AdmittanceRos::saturateAcc(const Vector6d &acc) const;
  // Vector6d AdmittanceRos::saturateVel(const Vector6d &vel) const;
  // bool AdmittanceRos::checkSuccess(const Eigen::Affine3d &desired_pose, const KDL::Frame &current_pose);
  // bool AdmittanceRos::checkAbsurdVelocities(const Vector6d &velocity) const;

  // ros::NodeHandle nh_;
  // std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
  // std::vector<std::string> eef_name_;
  // std::vector<Eigen::Affine3d> desired_pose_;
  // std::vector<Vector6d> cart_vel_;
  // Eigen::Matrix<double, 6, 6> B_, K_d_;
  // Eigen::MatrixXd K_p_;
  // bool use_right_, use_left_, avoid_joint_limits_; //TODO USE RIGHT OR LEFT? ARE YOU ROOT?
  // double force_dead_zone_, torque_dead_zone_, pos_offset_, max_lin_acc_, max_ang_acc_, max_lin_vel_, max_ang_vel_, linear_threshold_, angular_threshold_;
  // generic_control_toolbox::WrenchManager wrench_manager_;
  // generic_control_toolbox::MatrixParser matrix_parser_;



//   // PLUGINLIB_EXPORT_CLASS(yumi_experiments::AdmittanceRos,
//   // controller_interface::MultiInterfaceController)
