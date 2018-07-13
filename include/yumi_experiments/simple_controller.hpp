#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/ros.h>
#include <ros/node_handle.h>

#include <std_msgs/Float64.h>



typedef hardware_interface::VelocityJointInterface JointVelocityInterface;

namespace simple_controller
{

  class SimpleController : public controller_interface::Controller<JointVelocityInterface>
  {
  public:
        SimpleController();
        virtual ~SimpleController();

        bool init(JointVelocityInterface* hw, ros::NodeHandle& n);
        void starting(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);
        void stopping(const ros::Time& time);
  private:
        std::vector<hardware_interface::JointHandle> velocityJointHandles;
        std::vector<double> buffer_command_velocity;
        std::vector<double> buffer_current_positions;
        std::vector<double> buffer_current_velocities;
        // std::vector<double> buffer_current_positions;
        ros::Publisher state_pub_;
        ros::NodeHandle n;

  };


}
#endif
