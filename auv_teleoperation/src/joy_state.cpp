/**
 * @file joy_state.cpp
 * @brief Joystick state class for sampling button and axis states.
 * @author Joan Pau Beltran
 * @date 2011-02-08
 *
 * This file defines the joystick state class to retrieve information
 * about the current state of axes and buttons and their changes.
 */

#include "fugu_teleoperation/joy_state.h"

/** Constructor to initialize members (not moved and zero position).
 *
 * @return
 */
fugu_teleoperation::JoyState::AxisState::AxisState()
: position_(0.0), moved_(false)
{}

/** Constructor to initialize members (not moved and zero position).
 *
 * @return
 */
fugu_teleoperation::JoyState::ButtonState::ButtonState()
: position_(0), moved_(false)
{}

/** Default constructor doing nothing.
 *
 * @return
 */
fugu_teleoperation::JoyState::JoyState()
: stamp_(0.0)
{}

/** Update axes and buttons state from message.
 *
 * If a button is pressed in the current state and is depressed in the message,
 * it goes released.
 * If a button is depressed in the current state and is pressed in the message,
 * it goes pressed.
 * If an axis has different positions in the current state and the message,
 * it goes moved.
 * If the number of buttons or axes in the current state does not match
 * the number of buttons or axes in the message,
 * new elements are added or existing ones are removed.
 *
 * @param joy_msg message to update the state from.
 */
void fugu_teleoperation::JoyState::update(const sensor_msgs::JoyConstPtr& joy_msg)
{
  const int num_axes = joy_msg->axes.size();
  axis_states_.resize(num_axes);
  for (int i=0; i<num_axes; i++)
  {
    if (axis_states_[i].position_ != (joy_msg->axes)[i])
    {
      axis_states_[i].position_ = (joy_msg->axes)[i];
      axis_states_[i].moved_ = true;
    }
    else if (axis_states_[i].moved_)
    {
      axis_states_[i].moved_ = false;
    }
  }

  const int num_buttons = joy_msg->buttons.size();
  button_states_.resize(num_buttons);
  for (int i=0; i<num_buttons; i++)
  {
    if (button_states_[i].position_ != (joy_msg->buttons)[i])
    {
      button_states_[i].position_ = (joy_msg->buttons)[i];
      button_states_[i].moved_ = true;
    }
    else if (button_states_[i].moved_)
    {
      button_states_[i].moved_ = false;
    }
  }

  stamp_ = ( joy_msg->header.stamp.isValid() )
              ? joy_msg->header.stamp.toSec()
              : ros::Time::now().toSec();
}

/** Check if a button index is valid in the current state.
 *
 * @param i button index (negative means invalid button).
 * @return true if index is non-negative and less than the number of buttons.
 */
bool fugu_teleoperation::JoyState::button(int i) const
{
  return (0<=i) && ((unsigned int)i<button_states_.size());
}

/** Get button position in last update.
 *
 * @param i button index.
 * @return button position (1 means pressed, 0 means depressed or invalid index).
 */
int fugu_teleoperation::JoyState::buttonPosition(int i) const
{
  if( button(i) )
    return button_states_[i].position_;
  else
    return 0;
}

/** Check if button has been moved in last update.
 *
 * @param i button index.
 * @return true if button is valid and moved, false otherwise.
 */
bool fugu_teleoperation::JoyState::buttonMoved(int i) const
{
  if( button(i) )
    return button_states_[i].moved_;
  else
    return false;
}


/** Check if button has been pressed in last update.
 *
 * @param i button index.
 * @return true if button is valid and moved and now is pressed, false otherwise.
 */
bool fugu_teleoperation::JoyState::buttonPressed(int i) const
{
  if( button(i) )
    return button_states_[i].moved_ && (button_states_[i].position_>0);
  else
    return false;
}

/** Check if button has been released in last update.
 *
 * @param i button index.
 * @return true if button is valid and moved and now is depressed, false otherwise.
 */
bool fugu_teleoperation::JoyState::buttonReleased(int i) const
{
  if( button(i) )
    return button_states_[i].moved_ && (button_states_[i].position_==0);
  else
    return false;
}

/** Check if an axis index is valid in the current state.
 *
 * @param i axis index (negative means invalid axis).
 * @return true if index is non-negative and less than the number of axes.
 */
bool fugu_teleoperation::JoyState::axis(int i) const
{
  return (0<=i) && ((unsigned int)i<axis_states_.size());
}


/** Get axis position in last update.
 *
 * @param i axis index.
 * @return axis position (in range [-1,1], 0.0 means center or invalid index).
 */
float fugu_teleoperation::JoyState::axisPosition(int i) const
{
  if( axis(i) )
    return axis_states_[i].position_;
  else
    return 0.0;
}

/** Check if axis has been moved in last update.
 *
 * @param i axis index.
 * @return true if axis is valid and moved, false otherwise.
 */
bool fugu_teleoperation::JoyState::axisMoved(int i) const
{
  if( axis(i) )
    return axis_states_[i].moved_;
  else
    return false;
}

/** Get state time stamp.
 *
 * @return time stamp of last sample.
 */
double fugu_teleoperation::JoyState::stamp() const
{
  return stamp_;
}
