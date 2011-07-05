/**
 * @file motor_policy.cpp
 * @brief Motor teleoperation policy presentation.
 * @author Joan Pau Beltran
 * @date 2011-06-29
 * 
 * Joystick commands are translated to motor speeds
 * and published on the corresponding topic.
 * Several button actions are implemented:
 *   - increase linear speed offset for each motor pair
 *   - decrease linear speed offset for each motor pair
 *   - increase angular speed offset for each motor pair
 *   - decrease angular speed offset for each motor pair
 *   - reset linear speed offset for each motor pair
 *   - reset angular speed offset for each motor pair
 *   - pause all motors (without resetting offsets)
 *
 * @par Published topics
 *
 *   - @b motor_levels (control_common/MotorLevelsStamped)
 *   motor levels with the current time stamp.
 *
 * @par Parameters
 *
 * - @b ~fw_lin_incrm_axis forward motor pair linear speed axis number
 * - @b ~fw_lin_poffs_bttn forward motor pair increase linear speed offset button number
 * - @b ~fw_lin_moffs_bttn forward motor pair decrease linear speed offset button number
 * - @b ~fw_lin_reset_bttn forward motor pair reset linear speed button number
 * - @b ~fw_ang_incrm_bttn forward motor pair angular speed axis number
 * - @b ~fw_ang_poffs_bttn forward motor pair increase angular speed offset button number
 * - @b ~fw_ang_moffs_bttn forward motor pair decrease angular speed offset button number
 * - @b ~fw_ang_reset_bttn forward motor pair reset angular speed button number
 *
 * - @b ~dw_lin_incrm_bttn downward motor pair linear speed axis number
 * - @b ~dw_lin_poffs_bttn downward motor pair increase linear speed offset button number
 * - @b ~dw_lin_moffs_bttn downward motor pair decrease linear speed offset button number
 * - @b ~dw_lin_reset_bttn downward motor pair reset linear speed button number
 * - @b ~dw_ang_incrm_bttn downward motor pair angular speed axis number
 * - @b ~dw_ang_poffs_bttn downward motor pair increase angular speed offset button number
 * - @b ~dw_ang_moffs_bttn downward motor pair decrease angular speed offset button number
 * - @b ~dw_ang_reset_bttn downward motor pair reset angular speed button number
 *
 * - @b ~pause_bttn pause all motors button number (offsets are preserved)
 *
 * - @b ~fw_lin_factor forward motor pair linear speed axis factor
 * - @b ~fw_lin_step forward motor pair linear speed offset step
 * - @b ~fw_ang_factor forward motor pair angular speed axis factor
 * - @b ~fw_ang_step forward motor pair angular speed offset step
 *
 * - @b ~dw_lin_factor downward motor pair linear speed axis factor
 * - @b ~dw_lin_step downward motor pair linear speed offset step
 * - @b ~dw_ang_factor downward motor pair angular speed axis factor
 * - @b ~dw_ang_step downward motor pair angular speed offset step
 *
 * - @b ~frame_id frame name for published messages
 */

#include "fugu_teleoperation/motor_policy.h"
#include <albatros_motorboard/motorboardctrl.h>
#include <control_common/control_types.h>
#include <string>


/** Constructor passing through the node handles
 *
 * The private policy namespace is "motor_policy" relative to p's namespace.
 * The policy namespace for publications is n's namespace. *
 *
 * @param n node namespace handle
 * @param p private namespace handle
 * @return
 */
fugu_teleoperation::MotorPolicy::MotorPolicy(const ros::NodeHandle& n,
                                             const ros::NodeHandle& p)
{
  nh_ = n;
  priv_ = ros::NodeHandle(p,"motor_policy");
}


/** Set parameters from parameter server.
 */
void fugu_teleoperation::MotorPolicy::initParams()
{
  ROS_INFO_STREAM("Setting motor policy mapping and parameters...");
  std::string pair_prefix[NUM_PAIRS];
  pair_prefix[FORWARD] = "fw";
  pair_prefix[DOWNWARD] = "dw";
  std::string speed_prefix[NUM_SPEEDS];
  speed_prefix[LINEAR] = "lin";
  speed_prefix[ANGULAR] = "ang";
  for (int p=0; p<NUM_PAIRS; p++)
    for (int s=0; s<NUM_SPEEDS; s++)
    {
      std::string prefix = pair_prefix[p] + "_" + speed_prefix[s];
      priv_.param(prefix+"_incrm_axis", pair_mappings_[p].speed_mappings_[s].incrm_axis_, -1);
      priv_.param(prefix+"_noffs_bttn", pair_mappings_[p].speed_mappings_[s].noffs_bttn_, -1);
      priv_.param(prefix+"_poffs_bttn", pair_mappings_[p].speed_mappings_[s].poffs_bttn_, -1);
      priv_.param(prefix+"_reset_bttn", pair_mappings_[p].speed_mappings_[s].reset_bttn_, -1);
      priv_.param(prefix+"_factor", pair_mappings_[p].speed_mappings_[s].factor_,0.0);
      priv_.param(prefix+"_step", pair_mappings_[p].speed_mappings_[s].step_, 0.0);

      ROS_DEBUG_STREAM(prefix+" increment axis   : " << pair_mappings_[p].speed_mappings_[s].incrm_axis_);
      ROS_DEBUG_STREAM(prefix+" increment factor : " << pair_mappings_[p].speed_mappings_[s].factor_);
      ROS_DEBUG_STREAM(prefix+" offset buttons : " << pair_mappings_[p].speed_mappings_[s].noffs_bttn_ << " "
                                                   << pair_mappings_[p].speed_mappings_[s].poffs_bttn_);
      ROS_DEBUG_STREAM(prefix+" offset step    : " << pair_mappings_[p].speed_mappings_[s].step_);
      ROS_DEBUG_STREAM(prefix+" reset button : " << pair_mappings_[p].speed_mappings_[s].reset_bttn_);
    }
    priv_.param("pause_bttn",pause_bttn_,-1);
    ROS_DEBUG_STREAM("Pause button set to " << pause_bttn_);

    priv_.param("frame_id",frame_id_,std::string(""));
    ROS_DEBUG_STREAM("Frame id set to " << frame_id_);
}


/** Advertise motor levels topic.
 */
void fugu_teleoperation::MotorPolicy::advertiseTopics()
{
  ROS_INFO_STREAM("Advertising teleoperation motor levels...");
  publ_ = nh_.advertise<control_common::MotorLevelsStamped>("motor_levels", 10);
}


/** TeleopPolicy init function redefinition.
 *
 * Initialize parameters from server and advertise motor levels topic.
 */
void fugu_teleoperation::MotorPolicy::init()
{
  // Set mapping and other parameters from parameter server
  initParams();

  // Advertise motor levels topic
  advertiseTopics();
}


/** TeleopPolicy start function redefinition.
 *
 * Reset motor levels to null state.
 */
void fugu_teleoperation::MotorPolicy::start()
{
  ROS_INFO_STREAM("Initializing motor policy states...");
  for (int i=0; i<NUM_PAIRS; i++)
  {
    for (int j=0; i<NUM_SPEEDS; i++)
    {
      pair_states_[i].speeds_[j].incrm_ = 0.0;
      pair_states_[i].speeds_[j].offst_ = 0.0;
      pair_states_[i].speeds_[j].value_ = 0.0;
    }
    for (int j=0; j<NUM_MOTORS; j++)
      pair_states_[i].motors_[j] = 0.0;
  }
  control_common::MotorLevelsStampedPtr msg(new control_common::MotorLevelsStamped());
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = frame_id_;
  msg->motor_levels.levels.resize(albatros_motorboard::MotorBoardCtrl::NUM_MOTORS,0.0);
  publ_.publish(msg);
}

/** Update pair state from joystick state.
 *
 * The left motor level is equal to linear speed plus angular speed.
 * The right motor level is equal to linear speed minus angular speed.
 *
 * @param p pair state to update.
 * @param m pair mapping.
 * @param j joystick state.
 * @return whether the pair state is modified by the joystick state.
 */
bool fugu_teleoperation::MotorPolicy::updatePairState(PairState& p,
                                                      const PairMapping& m,
                                                      const JoyState& j)
{
  bool pair_update = false;
  for (int i=0; i<NUM_SPEEDS; i++)
  {
    bool update = false;
    if ( j.buttonPressed(m.speed_mappings_[i].reset_bttn_) )
    {
      p.speeds_[i].offst_ = 0.0;
      update = true;
    }
    if ( j.buttonPressed(m.speed_mappings_[i].noffs_bttn_) )
    {
      p.speeds_[i].offst_ -= m.speed_mappings_[i].step_;
      update = true;
    }
    if ( j.buttonPressed(m.speed_mappings_[i].poffs_bttn_) )
    {
      p.speeds_[i].offst_ += m.speed_mappings_[i].step_;
      update = true;
    }
    if ( j.axisMoved(m.speed_mappings_[i].incrm_axis_) )
    {
      p.speeds_[i].incrm_ = m.speed_mappings_[i].factor_ * j.axisPosition(m.speed_mappings_[i].incrm_axis_);
      update = true;
    }
    if (update)
    {
      p.speeds_[i].value_ = p.speeds_[i].offst_ + p.speeds_[i].incrm_;
      pair_update = true;
    }
  }
  if (pair_update)
  {
    p.motors_[LEFT] = p.speeds_[LINEAR].value_ + p.speeds_[ANGULAR].value_;
    p.motors_[RIGHT] = p.speeds_[LINEAR].value_ - p.speeds_[ANGULAR].value_;
  }
  return pair_update;
}


/** TeleopPolicy update function redefinition.
 *
 * Implemented button actions are:
 *   - increase linear speed offset for each motor pair
 *   - decrease linear speed offset for each motor pair
 *   - increase agnular speed offset for each motor pair
 *   - decrease angular speed offset for each motor pair
 *   - reset linear speed offset for each motor pair
 *   - reset angular speed offset for each motor pair
 *   - pause all motors (without resetting offsets)
 * If any pair state is modified by the joystick,
 * the state is updated and a new motor levels message is published.
 *
 * @param j joystick state.
 */
void fugu_teleoperation::MotorPolicy::update(const JoyState& j)
{
  bool updated = false;
  for (int i=0; i<NUM_PAIRS; i++)
    if ( updatePairState(pair_states_[i],pair_mappings_[i],j) )
      updated = true;

  bool pause = j.buttonPressed(pause_bttn_);
  if ( pause )
  {
    control_common::MotorLevelsStampedPtr msg(new control_common::MotorLevelsStamped());
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = frame_id_;
    msg->motor_levels.levels.resize(albatros_motorboard::MotorBoardCtrl::NUM_MOTORS,0.0);
    publ_.publish(msg);
  }
  else if ( updated )
  {
    control_common::MotorLevelsStampedPtr msg(new control_common::MotorLevelsStamped());
    msg->header.stamp = ros::Time::now();
    msg->motor_levels.levels.resize(albatros_motorboard::MotorBoardCtrl::NUM_MOTORS);
    msg->motor_levels.levels[albatros_motorboard::MotorBoardCtrl::FORWARD_LEFT] =
        pair_states_[FORWARD].motors_[LEFT];
    msg->motor_levels.levels[albatros_motorboard::MotorBoardCtrl::FORWARD_RIGHT] =
        pair_states_[FORWARD].motors_[RIGHT];
    msg->motor_levels.levels[albatros_motorboard::MotorBoardCtrl::DOWNWARD_LEFT] =
        pair_states_[DOWNWARD].motors_[LEFT];
    msg->motor_levels.levels[albatros_motorboard::MotorBoardCtrl::DOWNWARD_RIGHT] =
        pair_states_[DOWNWARD].motors_[RIGHT];
    publ_.publish(msg);
  }
}

/** TeleopPolicy stop function redefinition.
 *
 * Send a motor levels message with null speeds for every motor.
 */
void fugu_teleoperation::MotorPolicy::stop()
{
  ROS_INFO_STREAM("Sending null command on motor policy stop...");
  control_common::MotorLevelsStampedPtr msg(new control_common::MotorLevelsStamped());
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = frame_id_;
  msg->motor_levels.levels.resize(albatros_motorboard::MotorBoardCtrl::NUM_MOTORS,0.0);
  publ_.publish(msg);
}


