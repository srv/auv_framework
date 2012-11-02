/**
 * @file wrench_policy.h
 * @brief Motor teleoperation policy presentation.
 * @author Joan Pau Beltran
 * @date 2011-06-29
 */

#ifndef MOTOR_POLICY_H
#define MOTOR_POLICY_H

#include <ros/ros.h>
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

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  ros::Publisher publ_;

  std::string frame_id_;

  enum Speed {LINEAR=0, ANGULAR};
  static const int NUM_SPEEDS = 2;

  enum Pair {FORWARD=0, DOWNWARD};
  static const int NUM_PAIRS = 2;

  enum Motor {LEFT=0, RIGHT};
  static const int NUM_MOTORS = 2;

  struct SpeedMapping
  {
    int incrm_axis_;
    int poffs_bttn_;
    int noffs_bttn_;
    int reset_bttn_;
    double factor_;
    double step_;
  };

  struct PairMapping
  {
    SpeedMapping speed_mappings_[NUM_SPEEDS];
    int pause_bttn_;
  };

  struct SpeedState
  {
    float offst_;
    float incrm_;
    float value_;
  };

  struct PairState
  {
    SpeedState speeds_[NUM_SPEEDS];
    double motors_[NUM_MOTORS];
  };

  PairMapping pair_mappings_[NUM_PAIRS];
  PairState pair_states_[NUM_PAIRS];
  int pause_bttn_;

  void initParams();
  void advertiseTopics();
  void initPairStates();
  bool updatePairState(PairState& p, const PairMapping& m,
                       const auv_teleoperation::JoyState& j);

};

} // namespace

#endif // MOTOR_POLICY_H
