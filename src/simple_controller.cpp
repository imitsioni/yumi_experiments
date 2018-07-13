#include <yumi_experiments/simple_controller.hpp>
#include <pluginlib/class_list_macros.h>


namespace simple_controller
{

 SimpleController::SimpleController()
   {
    n = ros::NodeHandle("~");
    // got_first_ = false;
    // state_pub_ = n.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_1_l/command", 1);
  }

   SimpleController::~SimpleController()
   {}

   bool SimpleController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& n)
   {
      velocityJointHandles.clear();
      std::vector<std::string> jointNames;

      if(n.getParam("joints", jointNames))
      {
          if(hw)
          {
             for(unsigned int i=0; i < jointNames.size(); ++i)
             {
                 velocityJointHandles.push_back(hw->getHandle(jointNames[i]));
             }

             buffer_command_velocity.resize(velocityJointHandles.size(), 0.0);
             buffer_current_positions.resize(velocityJointHandles.size(), 0.0);
             buffer_current_velocities.resize(velocityJointHandles.size(), 0.0);
          }
          else
          {
              ROS_ERROR("Effort Joint Interface is empty in hardware interace.");
              return false;
          }
       }
       else
       {
          ROS_ERROR("No joints in given namespace: %s", n.getNamespace().c_str());
          ROS_INFO("yo");
          return false;
       }

       return true;
   }

   void SimpleController::starting(const ros::Time& time)
   {
      for(unsigned int i = 0; i < velocityJointHandles.size(); ++i)
      {
          buffer_current_positions[i] = velocityJointHandles[i].getPosition();
          buffer_current_velocities[i] = velocityJointHandles[i].getVelocity();
      }
   }

   void SimpleController::update(const ros::Time& time, const ros::Duration& period)
   {
      for(unsigned int i = 0; i < velocityJointHandles.size(); ++i)
      {
          buffer_current_positions[i] = velocityJointHandles[i].getPosition();
          buffer_current_velocities[i] = velocityJointHandles[i].getVelocity();
          buffer_command_velocity[i] = -0.5* buffer_current_velocities[i];
          // velocityJointHandles[i].setCommand(buffer_command_velocity[i]);
          // joint_command_pub_ = n.advertise<std_msgs::Float64>("/yumi/joint_vel_controller_1_l/command", 10);
      }
   }

   void SimpleController::stopping(const ros::Time& time)
   {}
}

PLUGINLIB_EXPORT_CLASS(simple_controller::SimpleController, controller_interface::ControllerBase)
