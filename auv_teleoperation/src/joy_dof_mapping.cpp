#include "auv_teleoperation/joy_dof_mapping.h"
#include "auv_teleoperation/parameter_helpers.h"

void auv_teleoperation::JoyDOFMapping::init(
    int num_dofs, const ros::NodeHandle& nh)
{
  assert(num_dofs > 0);

  ROS_INFO_STREAM("Loading DOF mappings...");

  std::vector<int> joy_axes = parameter_helpers::read_param_list<int>(
      nh, "joy_axes", XmlRpc::XmlRpcValue::TypeInt);
  std::vector<int> positive_offset_buttons = 
    parameter_helpers::read_param_list<int>(
      nh, "positive_offset_buttons", XmlRpc::XmlRpcValue::TypeInt);
  std::vector<int> negative_offset_buttons = 
    parameter_helpers::read_param_list<int>(
      nh, "negative_offset_buttons", XmlRpc::XmlRpcValue::TypeInt);
  std::vector<int> offset_reset_buttons = 
    parameter_helpers::read_param_list<int>(
      nh, "offset_reset_buttons", XmlRpc::XmlRpcValue::TypeInt);
  std::vector<double> joy_axes_factors = 
    parameter_helpers::read_param_list<double>(
      nh, "joy_axes_factors", XmlRpc::XmlRpcValue::TypeDouble);
  std::vector<double> offset_steps = 
    parameter_helpers::read_param_list<double>(
      nh, "offset_steps", XmlRpc::XmlRpcValue::TypeDouble);
  
  joy_dof_settings_.resize(num_dofs);
  if (joy_axes.size() != static_cast<size_t>(num_dofs))
    throw ParameterError(
        "Parameter joy_axes has wrong number of elements!");
  if (positive_offset_buttons.size() != static_cast<size_t>(num_dofs))
    throw ParameterError(
        "Parameter positive_offset_buttons has wrong number of elements!");
  if (negative_offset_buttons.size() != static_cast<size_t>(num_dofs))
    throw ParameterError(
        "Parameter negative_offset_buttons has wrong number of elements!");
  if (offset_reset_buttons.size() != static_cast<size_t>(num_dofs))
    throw ParameterError(
        "Parameter offset_reset_buttons: wrong number of elements!");
  if (joy_axes_factors.size() != static_cast<size_t>(num_dofs))
    throw ParameterError(
        "Parameter joy_axes_factors has wrong number of elements!");
  if (offset_steps.size() != static_cast<size_t>(num_dofs))
    throw ParameterError(
        "Parameter offset_steps has wrong number of elements!");
  for (int i = 0; i < num_dofs; ++i)
  {
    joy_dof_settings_[i].joy_axis = joy_axes[i];
    joy_dof_settings_[i].negative_offset_button = 
      negative_offset_buttons[i];
    joy_dof_settings_[i].positive_offset_button = 
      positive_offset_buttons[i];
    joy_dof_settings_[i].offset_reset_button = offset_reset_buttons[i];
    joy_dof_settings_[i].axis_factor = joy_axes_factors[i];
    joy_dof_settings_[i].offset_step = offset_steps[i];
  }
}

bool auv_teleoperation::JoyDOFMapping::update(
    const JoyState& joy_state, std::vector<DOFState>& dof_states)
{
  assert(dof_states.size() == joy_dof_settings_.size());
  bool updated = false;
  for (size_t i = 0; i < joy_dof_settings_.size(); ++i)
  {
    dof_states[i].updated = false;
    if (joy_state.buttonPressed(
          joy_dof_settings_[i].negative_offset_button))
    {
      dof_states[i].offset -= joy_dof_settings_[i].offset_step;
      dof_states[i].updated = true;
      updated = true;
    }
    if (joy_state.buttonPressed(
          joy_dof_settings_[i].positive_offset_button))
    {
      dof_states[i].offset += joy_dof_settings_[i].offset_step;
      dof_states[i].updated = true;
      updated = true;
    }
    if (joy_state.buttonPressed(joy_dof_settings_[i].offset_reset_button))
    {
      dof_states[i].offset = 0.0;
      dof_states[i].updated = true;
      updated = true;
    }
    if (joy_state.axisMoved(joy_dof_settings_[i].joy_axis))
    {
      dof_states[i].value = joy_dof_settings_[i].axis_factor * 
        joy_state.axisPosition(joy_dof_settings_[i].joy_axis);
      dof_states[i].updated = true;
      updated = true;
    }
  }
  return updated;
}

