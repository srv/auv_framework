#include <string>
#include <auv_control_msgs/MotorLevels.h>
#include "auv_teleoperation/motor_policy.h"
#include "auv_teleoperation/parameter_helpers.h"

auv_teleoperation::MotorPolicy::MotorPolicy(
    ros::NodeHandle& nh, const ros::NodeHandle& nh_priv) :
  TeleoperationPolicy(6, nh_priv), nh_(nh), nh_priv_(nh_priv)
{
  nh_priv_.param("frame_id", frame_id_, std::string("base_link"));
  ROS_DEBUG_STREAM("Frame id set to " << frame_id_);

  std::vector<double> axes_to_motors = 
    parameter_helpers::read_param_list<double>(
      nh_priv_, "axes_to_motors", XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(axes_to_motors.size() % 12 == 0);
  int num_motors = axes_to_motors.size() / 12;
  axes_to_motors_.resize(num_motors, 12);
  for (int r = 0; r < num_motors; ++r)
    for (int c = 0; c < 12; ++c)
      axes_to_motors_(r, c) = axes_to_motors[r * 12 + c];
  ROS_DEBUG_STREAM("axes_to_motors set to " << axes_to_motors_);

  ROS_INFO_STREAM("Advertising teleoperation motor levels as " <<
      nh_priv_.resolveName("motor_levels"));
  pub_ = nh_priv_.advertise<auv_control_msgs::MotorLevels>(
      "motor_levels", 1);
}

void auv_teleoperation::MotorPolicy::start()
{
  ROS_INFO_STREAM("Starting motor policy...");
  resetDOFStates();
  updateDOFs(ros::Time::now());
}

void auv_teleoperation::MotorPolicy::pause()
{
  ROS_INFO_STREAM("Sending null command on motor policy pause...");
  auv_control_msgs::MotorLevels msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  msg.levels.resize(axes_to_motors_.rows(), 0.0);
  pub_.publish(msg);
}

void auv_teleoperation::MotorPolicy::stop()
{
  ROS_INFO_STREAM("Sending null command on motor policy stop...");
  resetDOFStates();
  updateDOFs(ros::Time::now());
}

void auv_teleoperation::MotorPolicy::updateDOFs(const ros::Time& stamp)
{
  Eigen::VectorXd dof_values_vec(12);
  dof_values_vec.fill(0.0);
  for (size_t i = 0; i < 6; ++i)
  {
    float dof_value = dof_states_[i].getValue();
    if (dof_value > 0)
    {
      dof_values_vec(2*i) = dof_value;
    }
    else
    {
      dof_values_vec(2*i+1) = -dof_value;
    }
  }
  Eigen::VectorXd motor_levels = axes_to_motors_ * dof_values_vec;

  auv_control_msgs::MotorLevels msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.levels.resize(axes_to_motors_.rows());
  for (size_t i = 0; i < msg.levels.size(); ++i)
  {
    msg.levels[i] = motor_levels(i);
  }
  pub_.publish(msg);
}

