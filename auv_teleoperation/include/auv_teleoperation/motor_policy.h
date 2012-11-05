/**
 * @file wrench_policy.h
 * @brief Motor teleoperation policy presentation.
 * @author Joan Pau Beltran, Stephan Wirth
 * @date 2011-06-29
 */

#ifndef MOTOR_POLICY_H
#define MOTOR_POLICY_H

#include <ros/ros.h>
#include <Eigen/Core>
#include "auv_teleoperation/teleop_policy.h"

namespace auv_teleoperation
{

/** Motor teleoperation policy class.
 *
 * The motor policy responds to joystick events
 * controlling the motor levels on each pair of motors.
 */
class MotorPolicy : public TeleopPolicy
{
public:
  MotorPolicy(const ros::NodeHandle& n, const ros::NodeHandle& p);
  void init();
  void update(const JoyState& j); //!< joystick response
  void start();
  void stop();

protected:
  void sendStopMessage(const ros::Time& stamp);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  ros::Publisher publ_;

  std::string frame_id_;
  int pause_button_;

  struct DOFState
  {
    float offset;
    float value;
  };

  struct JoyDOFSetting
  {
    int joy_axis;
    int negative_offset_button;
    int positive_offset_button;
    int reset_button;
    float axis_factor;
    float offset_step;
  };

  JoyDOFSetting joy_dof_settings_[6];
  DOFState dof_states_[6];

  Eigen::MatrixXd axes_to_motors_;
};

} // namespace

#endif // MOTOR_POLICY_H
