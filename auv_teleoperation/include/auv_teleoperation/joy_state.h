/**
 * @file joy_state.h
 * @brief Joystick state class for sampling button and axis states.
 * @author Joan Pau Beltran, Stephan Wirth
 *
 * This file presents a joystick state class to retrieve information
 * about the current state of axes and buttons and their changes.
 */

#ifndef JOYSTATE_H
#define JOYSTATE_H

#include <vector>
#include <sensor_msgs/Joy.h>

namespace auv_teleoperation
{

/**
 * @brief Joystick state class
 * This class represents the state of a joystick. It stores
 * axes and button positions and boolean flags that mark if an
 * axis or a button has been moved since the last update.
 * The update is done by means of a sensor_msgs::Joy message.
 */
class JoyState
{
public:
  /** 
   * Default constructor initializes the stamp to zero.
   */
  JoyState();

  /** Update axes and buttons state from message.
  *
  * If a button is pressed in the current state and is depressed 
  * in the message, it goes to state released.
  * If a button is depressed in the current state and is pressed 
  * in the message, it goes to state pressed.
  * If an axis has different positions in the current state than in the 
  * message, it goes to state moved.
  * If the number of buttons or axes in the current state does not match
  * the number of buttons or axes in the message,
  * new elements are added or existing ones are removed.
  *
  * @param joy_msg message to update the state from.
  */
  void update(const sensor_msgs::JoyConstPtr& joy_msg);

  /** 
   * Check if a button index is valid in the current state.
   * @param i button index
   * @return true if index is non-negative and less than the number 
   * of buttons.
   */
  inline bool hasButton(int i) const
  {
    return (0<=i) && ((unsigned int)i<button_states_.size());
  }

  /** Get button position from last update.
  *
  * @param i button index.
  * @return button position (1 means pressed, 0 means depressed or invalid index)
  */
  inline int buttonPosition(int i) const
  {
    return hasButton(i) ? button_states_[i].position : 0;
  }

  /** Check if button has been moved in last update.
  *
  * @param i button index.
  * @return true if button is valid and has been moved, false otherwise.
  */
  inline bool buttonMoved(int i) const
  {
    return hasButton(i) ? button_states_[i].moved : false;
  }

  /** Check if button has been pressed in last update.
  *
  * @param i button index.
  * @return true if button is valid and has been moved and now is pressed, 
  *         false otherwise.
  */
  inline bool buttonPressed(int i) const
  {
    return hasButton(i) 
      ? button_states_[i].moved && (button_states_[i].position>0)
      : false;
  }

  /** Check if button has been released in last update.
  *
  * @param i button index.
  * @return true if button is valid and has been moved and now is depressed, 
  *         false otherwise.
  */
  inline bool buttonReleased(int i) const
  {
    return hasButton(i)
      ? button_states_[i].moved && (button_states_[i].position==0)
      : false;
  }

  /** Check if an axis index is valid in the current state.
  *
  * @param i axis index
  * @return true if index is non-negative and less than the number of axes.
  */
  inline bool hasAxis(int i) const
  {
    return (0<=i) && ((unsigned int)i<axis_states_.size());
  }

  /** Get axis position in last update.
  *
  * @param i axis index. Must be valid.
  * @return axis position (in range [-1,1], 0.0 means center or invalid axis).
  */
  inline float axisPosition(int i) const
  {
    return hasAxis(i) ? axis_states_[i].position : 0.0;
  }

  /** Check if axis has been moved in last update.
  *
  * @param i axis index. Must be valid.
  * @return true if axis is valid and has been moved in last update, false 
  *         otherwise.
  */
  inline bool axisMoved(int i) const
  {
    return hasAxis(i) ? axis_states_[i].moved : false;
  }

  /**
   * @return last timestamp of received joy message
   */
  inline ros::Time stamp() const {
    return stamp_;
  }

private:

  /**
   * Stores the state of a joystick axis.
   */
  struct AxisState
  {
    float position;
    bool moved;
    inline AxisState() : position(0.0), moved(false) {};
  };

  /**
   * Stores the state of a joystick button.
   */
  struct ButtonState
  {
    int position; // 1=pressed, 0=released
    bool moved;
    inline ButtonState() : position(0), moved(false) {};
  };

  std::vector<AxisState> axis_states_;
  std::vector<ButtonState> button_states_;
  ros::Time stamp_; // time of the last joystick message

};

} // namespace

#endif // JOYSTATE_H
