#ifndef __ADMITTANCE_ROS__
#define __ADMITTANCE_ROS__


// #include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>

#include <generic_control_toolbox/kdl_manager.hpp>
#include <generic_control_toolbox/wrench_manager.hpp>
#include <generic_control_toolbox/ArmInfo.h>
#include <generic_control_toolbox/matrix_parser.hpp>


// namespace yumi_experiments
// {
  typedef hardware_interface::VelocityJointInterface JointVelocityInterface;
  typedef hardware_interface::ForceTorqueSensorInterface ForceTorqueSensorInterface;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  class AdmittanceRos : public controller_interface::MultiInterfaceController <JointVelocityInterface,
                                                                               ForceTorqueSensorInterface>
  {
  public:
    AdmittanceRos();
    ~AdmittanceRos();
    // bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &n);
    bool init();
    // void update(const ros::Time &time, const ros::Duration &period);
    void update();


  private:
    bool parseGoal(); // TODO maybe get the trajectory points here
    void computeAdmittanceError(const KDL::Frame &desired_pose, const KDL::Frame &pose, Vector6d &error) const;
    Vector6d computeCartesianAccelerations(const Vector6d &vel_eig, const KDL::Frame &pose, const Eigen::Affine3d &desired_pose, const Vector6d &wrench) const;
    Vector6d applyDeadZone(const Vector6d &wrench) const;
    Vector6d saturateAcc(const Vector6d &acc) const;
    Vector6d saturateVel(const Vector6d &vel) const;
    bool checkSuccess(const Eigen::Affine3d &desired_pose, const KDL::Frame &current_pose);
    bool checkAbsurdVelocities(const Vector6d &velocity) const;

    ros::NodeHandle nh_;
    ros::Publisher joint_command_pub_;
    std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
    std::vector<std::string> eef_name_;
    std::vector<Eigen::Affine3d> desired_pose_;
    std::vector<Vector6d> cart_vel_;
    Eigen::Matrix<double, 6, 6> B_, K_d_;
    Eigen::MatrixXd K_p_;
    bool use_right_, use_left_, avoid_joint_limits_;
    double force_dead_zone_, torque_dead_zone_, pos_offset_, max_lin_acc_, max_ang_acc_, max_lin_vel_, max_ang_vel_, linear_threshold_, angular_threshold_;
    generic_control_toolbox::WrenchManager wrench_manager_;
    generic_control_toolbox::MatrixParser matrix_parser_;

  };
// }
#endif
