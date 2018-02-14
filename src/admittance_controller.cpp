#include <yumi_experiments/admittance_controller.hpp>

namespace yumi_experiments
{
  AdmittanceController::AdmittanceController(const std::string action_name) : ControllerTemplate<AdmittanceControllerAction,
                                                                          AdmittanceControllerGoal,
                                                                          AdmittanceControllerFeedback,
                                                                          AdmittanceControllerResult>(action_name)
  {
    nh_ = ros::NodeHandle("~");

    if (!init())
    {
      throw std::logic_error("Missing parameters for the admittance controller");
    }
  }

  AdmittanceController::~AdmittanceController() {}

  bool AdmittanceController::init()
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

    if(!matrix_parser.parseMatrixData(K_p_, "admittance/K_p", nh_))
    {
      return false;
    }

    if (K_p_.rows() != 6 || K_p_.cols() != 6)
    {
      ROS_ERROR("Invalid K_p dimensions. Must be 6x6");
      return false;
    }

    frequency = settling_time*damping_ratio/4;
    B_ = K_p_/(frequency*frequency);
    K_d_ = 2*B_*damping_ratio*frequency;

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

  bool AdmittanceController::parseGoal(boost::shared_ptr<const AdmittanceControllerGoal> goal)
  {
    tf::poseMsgToEigen(goal->desired_right_pose.pose, desired_pose_[RIGHT_ARM]);
    tf::poseMsgToEigen(goal->desired_left_pose.pose, desired_pose_[LEFT_ARM]);
    cart_vel_[RIGHT_ARM] = Eigen::Matrix<double, 6, 1>::Zero();
    cart_vel_[LEFT_ARM] = Eigen::Matrix<double, 6, 1>::Zero();
    return true;
  }

  void AdmittanceController::resetController()
  {

  }

  sensor_msgs::JointState AdmittanceController::controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt)
  {
    sensor_msgs::JointState ret = current_state;
    std::vector<KDL::Frame> pose(2);
    std::vector<KDL::Twist> vel(2), desired_vel(2);
    std::vector<Eigen::Affine3d> pose_eig(2);
    std::vector<Eigen::Matrix<double, 6, 1> > vel_eig(2);
    std::vector<Eigen::Matrix<double, 6, 1> > wrench(2);
    std::vector<Eigen::Matrix<double, 6, 1> > acc(2);
    std::vector<Eigen::Matrix<double, 6, 1> > error(2);

    kdl_manager_->getGrippingPoint(eef_name_[LEFT_ARM], current_state, pose[LEFT_ARM]);
    kdl_manager_->getGrippingPoint(eef_name_[RIGHT_ARM], current_state, pose[RIGHT_ARM]);
    tf::transformKDLToEigen(pose[LEFT_ARM], pose_eig[LEFT_ARM]);
    tf::transformKDLToEigen(pose[RIGHT_ARM], pose_eig[RIGHT_ARM]);
    kdl_manager_->getGrippingTwist(eef_name_[LEFT_ARM], current_state, vel[LEFT_ARM]);
    kdl_manager_->getGrippingTwist(eef_name_[RIGHT_ARM], current_state, vel[RIGHT_ARM]);
    tf::twistKDLToEigen(vel[LEFT_ARM], vel_eig[LEFT_ARM]);
    tf::twistKDLToEigen(vel[RIGHT_ARM], vel_eig[RIGHT_ARM]);
    wrench_manager_.wrenchAtGrippingPoint(eef_name_[RIGHT_ARM], wrench[RIGHT_ARM]);
    wrench_manager_.wrenchAtGrippingPoint(eef_name_[LEFT_ARM], wrench[LEFT_ARM]);

    error[LEFT_ARM].block<3, 1>(0, 0) = desired_pose_[LEFT_ARM].translation() - pose_eig[LEFT_ARM].translation();
    error[LEFT_ARM].block<3, 1>(3, 0) = Eigen::Vector3d::Zero();

    B_ = Eigen::Matrix<double, 6, 6>::Identity();
    acc[LEFT_ARM] = B_.llt().solve(0*wrench[LEFT_ARM] - K_d_*0*vel_eig[LEFT_ARM] - K_p_*error[LEFT_ARM]);
    cart_vel_[LEFT_ARM] += acc[LEFT_ARM]*dt.toSec();

    std::cout << cart_vel_[LEFT_ARM] << std::endl << std::endl;
    std::cout << "-----------" << std::endl;
    tf::twistEigenToKDL(cart_vel_[LEFT_ARM], desired_vel[LEFT_ARM]);
    std::cout << desired_vel[LEFT_ARM].vel.x() << ", " << desired_vel[LEFT_ARM].vel.y() << ", " << desired_vel[LEFT_ARM].vel.z() << std::endl;

    KDL::JntArray q_dot(7);
    kdl_manager_->getGrippingVelIK(eef_name_[LEFT_ARM], current_state, desired_vel[LEFT_ARM], q_dot);
    kdl_manager_->getJointState(eef_name_[LEFT_ARM], q_dot.data, ret);

    return ret;
  }
}