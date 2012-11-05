/**
 * @file motor_policy.cpp
 * @brief Motor teleoperation policy presentation.
 * @author Joan Pau Beltran, Stephan Wirth
 * @date 2012-11-02
 */

#include <string>
#include <auv_control_msgs/MotorLevels.h>
#include "auv_teleoperation/motor_policy.h"

auv_teleoperation::MotorPolicy::MotorPolicy(const ros::NodeHandle& n,
                                             const ros::NodeHandle& p)
{
  nh_ = n;
  priv_ = ros::NodeHandle(p,"motor_policy");
}

/// helper for reading uniform parameter lists from the parameter server
template <typename T>
std::vector<T> read_param_list(const ros::NodeHandle& nh, const std::string& name, XmlRpc::XmlRpcValue::Type type)
{ 
  XmlRpc::XmlRpcValue list;
  nh.getParam(name, list);
  ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  std::vector<T> values(list.size());
  for (int i = 0; i < 6; ++i) 
  {
    ROS_ASSERT(list[i].getType() == type);
    values[i] = static_cast<T>(list[i]);
  }
  return values;
}

void auv_teleoperation::MotorPolicy::init()
{
  ROS_INFO_STREAM("Setting motor policy mapping and parameters...");

  std::vector<int> joy_axes = read_param_list<int>(
      priv_, "joy_axes", XmlRpc::XmlRpcValue::TypeInt);
  ROS_ASSERT(joy_axes.size() == 6);
  std::vector<int> positive_offset_buttons = read_param_list<int>(
      priv_, "positive_offset_buttons", XmlRpc::XmlRpcValue::TypeInt);
  ROS_ASSERT(positive_offset_buttons.size() == 6);
  std::vector<int> negative_offset_buttons = read_param_list<int>(
      priv_, "negative_offset_buttons", XmlRpc::XmlRpcValue::TypeInt);
  ROS_ASSERT(negative_offset_buttons.size() == 6);
  std::vector<int> reset_buttons = read_param_list<int>(
      priv_, "reset_buttons", XmlRpc::XmlRpcValue::TypeInt);
  ROS_ASSERT(reset_buttons.size() == 6);
  std::vector<double> joy_axes_factors = read_param_list<double>(
      priv_, "joy_axes_factors", XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(joy_axes_factors.size() == 6);
  std::vector<double> offset_steps = read_param_list<double>(
      priv_, "offset_steps", XmlRpc::XmlRpcValue::TypeDouble);
  ROS_ASSERT(offset_steps.size() == 6);
  for (int i = 0; i < 6; ++i)
  {
    joy_dof_settings_[i].joy_axis = joy_axes[i];
    joy_dof_settings_[i].negative_offset_button = negative_offset_buttons[i];
    joy_dof_settings_[i].positive_offset_button = positive_offset_buttons[i];
    joy_dof_settings_[i].reset_button = reset_buttons[i];
    joy_dof_settings_[i].axis_factor = joy_axes_factors[i];
    joy_dof_settings_[i].offset_step = offset_steps[i];
  }

  priv_.param("pause_button", pause_button_, -1);
  ROS_DEBUG_STREAM("Pause button set to " << pause_button_);

  priv_.param("frame_id", frame_id_, std::string("base_link"));
  ROS_DEBUG_STREAM("Frame id set to " << frame_id_);

  std::vector<double> axes_to_motors = read_param_list<double>(
      priv_, "axes_to_motors", XmlRpc::XmlRpcValue::TypeDouble);
  axes_to_motors_.resize(axes_to_motors.size(), 1);
  for (size_t i = 0; i < axes_to_motors.size(); ++i)
    axes_to_motors_(i) = axes_to_motors[i];
  axes_to_motors_.resize(axes_to_motors.size() / 12, 12);

  ROS_INFO_STREAM("Advertising teleoperation motor levels...");
  publ_ = nh_.advertise<auv_control_msgs::MotorLevels>("motor_levels", 1);
}

void auv_teleoperation::MotorPolicy::start()
{
  ROS_INFO_STREAM("Initializing motor policy states...");
  for (int i = 0; i < 6; ++i)
  {
    dof_states_[i].value = 0.0;
    dof_states_[i].offset = 0.0;
  }
  sendStopMessage(ros::Time::now());
}

void auv_teleoperation::MotorPolicy::update(const JoyState& j)
{
  bool updated = false;
  for (int i = 0; i < 6; ++i)
  {
    if (j.buttonPressed(joy_dof_settings_[i].negative_offset_button))
    {
      dof_states_[i].offset -= joy_dof_settings_[i].offset_step;
      updated = true;
    }
    if (j.buttonPressed(joy_dof_settings_[i].positive_offset_button))
    {
      dof_states_[i].offset += joy_dof_settings_[i].offset_step;
      updated = true;
    }
    if (j.buttonPressed(joy_dof_settings_[i].reset_button))
    {
      dof_states_[i].offset = 0.0;
      updated = true;
    }
    if (j.axisMoved(joy_dof_settings_[i].joy_axis))
    {
      dof_states_[i].value = joy_dof_settings_[i].axis_factor * j.axisPosition(joy_dof_settings_[i].joy_axis);
      updated = true;
    }
  }

  bool paused = j.buttonPressed(pause_button_);
  if (paused)
  {
    sendStopMessage(ros::Time(j.stamp()));
  }
  else if (updated)
  {
    Eigen::VectorXd dof_values(12);
    for (int i = 0; i < 6; ++i)
    {
      if (dof_states_[i].value > 0)
      {
        dof_values(2*i) = dof_states_[i].value;
      }
      else
      {
        dof_values(2*i+1) = -dof_states_[i].value;
      }
    }
    Eigen::VectorXd motor_levels = axes_to_motors_ * dof_values;

    auv_control_msgs::MotorLevelsPtr msg(new auv_control_msgs::MotorLevels());
    msg->header.stamp = ros::Time(j.stamp());
    msg->levels.resize(axes_to_motors_.rows());
    for (size_t i = 0; i < msg->levels.size(); ++i)
    {
      msg->levels[i] = motor_levels(i);
    }
  }
}

void auv_teleoperation::MotorPolicy::sendStopMessage(const ros::Time& stamp)
{
  auv_control_msgs::MotorLevelsPtr msg(new auv_control_msgs::MotorLevels());
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = frame_id_;
  msg->levels.resize(axes_to_motors_.rows(),0.0);
  publ_.publish(msg);
}

void auv_teleoperation::MotorPolicy::stop()
{
  ROS_INFO_STREAM("Sending null command on motor policy stop...");
  sendStopMessage(ros::Time::now());
}

