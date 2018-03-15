#ifndef __CALIBRATION_CONTROLLER__
#define __CALIBRATION_CONTROLLER__

#include <ros/ros.h>
#include <stdexcept>
#include <yumi_experiments/SensorCalibrationAction.h>
#include <generic_control_toolbox/kdl_manager.hpp>
#include <generic_control_toolbox/wrench_manager.hpp>
#include <generic_control_toolbox/controller_template.hpp>
#include <generic_control_toolbox/ArmInfo.h>
#include <generic_control_toolbox/matrix_parser.hpp>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

namespace yumi_experiments
{
  /**
    Implements a force-torque sensor calibration routine. A probe arm will move on
    a pre-defined trajectory along a case grasped by the arm with the probe to be calibrated.
    The controller will publish feedback information which can be used to compute a calibration
    matrix for the case sensor, so that the measured torque matches the expected r.cross(f).
  **/
  class SensorCalibrationController : public generic_control_toolbox::ControllerTemplate<SensorCalibrationAction,
                                                                                  SensorCalibrationGoal,
                                                                                  SensorCalibrationFeedback,
                                                                                  SensorCalibrationResult>
  {
  public:
    SensorCalibrationController(const std::string action_name);
    virtual ~SensorCalibrationController();

  private:
    sensor_msgs::JointState controlAlgorithm(const sensor_msgs::JointState &current_state, const ros::Duration &dt);
    bool parseGoal(boost::shared_ptr<const SensorCalibrationGoal> goal);
    bool init();
    void resetController();
    bool setArm(const std::string &arm_name, std::string &eef_name, std::string &sensor_frame);
    /**
      Compute the desired probe twist given the configured probe trajectory.
    **/
    KDL::Twist computeCommandTwist(const KDL::Frame &p_probe, const KDL::Frame &p_case, const KDL::Wrench &wrench_probe) const;
    Eigen::Vector3d getDir(const char dir, const int sign, const Eigen::Affine3d &p) const;
    bool setDirVars(char &dir, int &sign, const std::string &s) const;

    ros::NodeHandle nh_;
    std::shared_ptr<generic_control_toolbox::KDLManager> kdl_manager_;
    generic_control_toolbox::WrenchManager wrench_manager_;
    generic_control_toolbox::MatrixParser matrix_parser_;
    std::string case_arm_eef_, probe_arm_eef_, case_sensor_frame_, probe_sensor_frame_;
    double probe_tip_offset_, time_corner1_, time_corner2_, time_corner3_, time_corner4_, force_1_, force_2_, force_3_, force_4_, K_force_, vd_;
    char f_, t1_, t2_;
    int t1_sign_, t2_sign_, f_sign_;
    ros::Time init_time_;
  };
}
#endif
