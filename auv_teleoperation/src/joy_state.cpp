/**
 * @file joy_state.cpp
 * @brief Joystick state class for sampling button and axis states.
 * @author Joan Pau Beltran, Stephan Wirth
 *
 * This file defines the joystick state class to retrieve information
 * about the current state of axes and buttons and their changes.
 */

#include "auv_teleoperation/joy_state.h"

auv_teleoperation::JoyState::JoyState()
: stamp_(0.0)
{}

void auv_teleoperation::JoyState::update(const sensor_msgs::JoyConstPtr& joy_msg)
{
  const int num_axes = joy_msg->axes.size();
  axis_states_.resize(num_axes);
  for (int i=0; i<num_axes; i++)
  {
    if (axis_states_[i].position != (joy_msg->axes)[i])
    {
      axis_states_[i].position = (joy_msg->axes)[i];
      axis_states_[i].moved = true;
    }
    else if (axis_states_[i].moved)
    {
      axis_states_[i].moved = false;
    }
  }

  const int num_buttons = joy_msg->buttons.size();
  button_states_.resize(num_buttons);
  for (int i=0; i<num_buttons; i++)
  {
    if (button_states_[i].position != (joy_msg->buttons)[i])
    {
      button_states_[i].position = (joy_msg->buttons)[i];
      button_states_[i].moved = true;
    }
    else if (button_states_[i].moved)
    {
      button_states_[i].moved = false;
    }
  }

  stamp_ = ( joy_msg->header.stamp.isValid() )
              ? joy_msg->header.stamp
              : ros::Time::now();
}

