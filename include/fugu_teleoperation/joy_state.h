/**
 * @file joy_state.h
 * @brief Joystick state class for sampling button and axis states.
 * @author Joan Pau Beltran
 * @date 2011-02-08
 *
 * This file presents a joystick state class to retrieve information
 * about the current state of axes and buttons and their changes.
 */

#ifndef JOYSTATE_H
#define JOYSTATE_H

#include <joy/Joy.h>
#include <vector>

namespace fugu_teleoperation
{

/**
 * @brief Joystick state class
 */
class JoyState
{
public:

  JoyState();

  void update(const joy::JoyConstPtr& joy_msg);

  bool button(int i) const;         // true if the index is a valid button
  int buttonPosition(int i) const;  // 0 (pressed) or 1 (depressed)
  bool buttonMoved(int i) const;    // button moved in last update
  bool buttonPressed(int i) const;  // button pressed in last update
  bool buttonReleased(int i) const; // button released in last update

  bool axis(int i) const;           // true if the index is a valid axis
  float axisPosition(int i) const;  // position in [-1,1]
  bool axisMoved(int i) const;      // axis moved in last update

private:

  struct AxisState
  {
    float position_;
    bool moved_;
    AxisState();
  };

  struct ButtonState
  {
    int position_;
    bool moved_;
    ButtonState();
  };

  std::vector<AxisState> axis_states_;
  std::vector<ButtonState> button_states_;

};

} // namespace

#endif // JOYSTATE_H
