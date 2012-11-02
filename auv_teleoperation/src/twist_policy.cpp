/**
 * @file twist_policy.cpp
 * @brief Twist teleoperation policy implementation.
 * @author Joan Pau Beltran
 * @date 2011-06-29
 *
 * Joystick commands are translated to twist levels
 * and published on the corresponding topic.
 * Several button actions are implemented:
 *   - increase velocity offset for each DOF
 *   - decrease velocity offset for each DOF
 *   - reset velocity offset for each DOF
 *   - set all DOFs to null state  (without resetting offsets)
 *
 * @par Published topics
 *
 *   - @b twist_levels (control_common/TwistLevelsStamped)
 *   twist levels with the current time stamp.
 *
 * @par Parameters
 *
 *   - @b ~(lin|ang)_[xyz]_incrm_axis velocity axis number
 *   - @b ~(lin|ang)_[xyz]_poffs_bttn increase velocity offset button number
 *   - @b ~(lin|ang)_[xyz]_moffs_bttn decrease velocity offset button number
 *   - @b ~(lin|ang)_[xyz]_reset_bttn reset velocity offset button number
 *
 *   - @b ~pause_bttn set all velocities to null (offsets are preserved)
 *
 *   - @b ~frame_id frame name for published messages
 */

#include "auv_teleoperation/twist_policy.h"
#include "control_common/control_types.h"
#include <string>


/** Constructor passing through the node handles
 *
 * The private policy namespace is "twist_policy" relative to p's namespace.
 * The policy namespace for publications is n's namespace. *
 *
 * @param n node namespace handle
 * @param p private namespace handle
 * @return
 */
auv_teleoperation::TwistPolicy::TwistPolicy(const ros::NodeHandle& n,
                                             const ros::NodeHandle& p)
{
  nh_ = n;
  priv_ = ros::NodeHandle(p,"twist_policy");
}


/** Set parameters from parameter server.
 */
void auv_teleoperation::TwistPolicy::initParams()
{
  ROS_INFO_STREAM("Setting twist policy mapping and parameters...");
  std::string dof_names[NUM_DOFS];
  dof_names[LIN_X] = "lin_x";
  dof_names[LIN_Y] = "lin_y";
  dof_names[LIN_Z] = "lin_z";
  dof_names[ANG_X] = "ang_x";
  dof_names[ANG_Y] = "ang_y";
  dof_names[ANG_Z] = "ang_z";
  for (int i=0; i<NUM_DOFS; i++)
  {
    std::string dof_name = dof_names[i];
    priv_.param(dof_name+"_incrm_axis",dof_map_[i].incrm_axis_,-1);
    priv_.param(dof_name+"_noffs_bttn",dof_map_[i].noffs_bttn_,-1);
    priv_.param(dof_name+"_poffs_bttn",dof_map_[i].poffs_bttn_,-1);
    priv_.param(dof_name+"_reset_bttn",dof_map_[i].reset_bttn_,-1);
    priv_.param(dof_name+"_factor", dof_map_[i].factor_,0.0);
    priv_.param(dof_name+"_step", dof_map_[i].step_,0.0);

    ROS_DEBUG_STREAM(dof_name+" increment axis   : " << dof_map_[i].incrm_axis_);
    ROS_DEBUG_STREAM(dof_name+" increment factor : " << dof_map_[i].factor_);
    ROS_DEBUG_STREAM(dof_name+" offset buttons : " << dof_map_[i].noffs_bttn_ << " "
                                                   << dof_map_[i].poffs_bttn_);
    ROS_DEBUG_STREAM(dof_name+" offset step    : " << dof_map_[i].step_);
    ROS_DEBUG_STREAM(dof_name+" reset button : " << dof_map_[i].reset_bttn_);
  }
  priv_.param("pause_bttn",pause_bttn_,-1);
  ROS_DEBUG_STREAM("Pause button set to " << pause_bttn_);

  priv_.param("frame_id",frame_id_,std::string(""));
  ROS_DEBUG_STREAM("Frame id set to " << frame_id_);

}


/** Advertise wrench levels topic.
 */
void auv_teleoperation::TwistPolicy::advertiseTopics()
{
  ROS_INFO_STREAM("Advertising teleoperation twist levels...");
  publ_ = nh_.advertise<control_common::TwistLevelsStamped>("twist_levels", 10);
}


/** TeleopPolicy init function redefinition
 *
 * Initialize parameters from server and advertise twist levels topic.
 */
void auv_teleoperation::TwistPolicy::init()
{
  // Set mapping and other parameters from parameter server
  initParams();

  // Advertise wrench levels topic
  advertiseTopics();
}


/** TeleopPolicy start function redefinition.
 *
 * Reset twist levels to null state.
 */
void auv_teleoperation::TwistPolicy::start()
{
  ROS_INFO_STREAM("Initializing twist policy states...");
  for (int i=0; i<NUM_DOFS; i++)
  {
    dof_state_[i].incrm_ = 0.0;
    dof_state_[i].offst_ = 0.0;
    dof_state_[i].value_ = 0.0;
  }
  control_common::TwistLevelsStampedPtr msg(new control_common::TwistLevelsStamped());
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = frame_id_;
  msg->twist.linear.x = 0.0;
  msg->twist.linear.y = 0.0;
  msg->twist.linear.z = 0.0;
  msg->twist.angular.x = 0.0;
  msg->twist.angular.y = 0.0;
  msg->twist.angular.z = 0.0;
  publ_.publish(msg);
}


/** Update DOFState from joystick state.
 *
 * @param d DOF state.
 * @param m DOF mapping.
 * @param j joystick state
 * @return whether the DOF state is modified by the joystick state.
 */
bool auv_teleoperation::TwistPolicy::updateDOFState(DOFState& d,
                                                      const DOFMapping& m,
                                                      const JoyState& j)
{
  bool update = false;
  if ( j.buttonPressed(m.reset_bttn_) )
  {
    d.offst_ = 0.0;
    update = true;
  }
  if ( j.buttonPressed(m.noffs_bttn_) )
  {
    d.offst_ -= m.step_;
    update = true;
  }
  if ( j.buttonPressed(m.poffs_bttn_) )
  {
    d.offst_ += m.step_;
    update = true;
  }
  if ( j.axisMoved(m.incrm_axis_) )
  {
    d.incrm_ = m.factor_*j.axisPosition(m.incrm_axis_);
    update = true;
  }
  if (update)
    d.value_ = d.offst_ + d.incrm_;
  return update;
}


/** TeleopPolicy update function redefinition.
 *
 * Implemented button actions are:
 *   - increase velocity offset for each DOF
 *   - decrease velocity offset for each DOF
 *   - reset velocity offset for each DOF
 *   - set all DOFs to null state  (without resetting offsets)
 * If any DOF twist level is modified by the joystick,
 * the DOF state is updated and a new twist levels message is published.
 *
 * @param j joystick state.
 */
void auv_teleoperation::TwistPolicy::update(const JoyState& j)
{
  bool updated = false;
  for (int i=0; i<NUM_DOFS; i++)
    if ( updateDOFState(dof_state_[i], dof_map_[i], j) )
      updated = true;

  bool pause = j.buttonPressed(pause_bttn_);
  if ( pause )
  {
    control_common::TwistLevelsStampedPtr msg(new control_common::TwistLevelsStamped());
    msg->header.stamp = ros::Time(j.stamp());
    msg->header.frame_id = frame_id_;
    msg->twist.linear.x = 0.0;
    msg->twist.linear.y = 0.0;
    msg->twist.linear.z = 0.0;
    msg->twist.angular.x = 0.0;
    msg->twist.angular.y = 0.0;
    msg->twist.angular.z = 0.0;
    publ_.publish(msg);
  }
  else if ( updated )
  {
    control_common::TwistLevelsStampedPtr msg(new control_common::TwistLevelsStamped());
    msg->header.stamp = ros::Time(j.stamp());
    msg->header.frame_id = frame_id_;
    msg->twist.linear.x = dof_state_[LIN_X].value_;
    msg->twist.linear.y = dof_state_[LIN_Y].value_;
    msg->twist.linear.z = dof_state_[LIN_Z].value_;
    msg->twist.angular.x = dof_state_[ANG_X].value_;
    msg->twist.angular.y = dof_state_[ANG_Y].value_;
    msg->twist.angular.z = dof_state_[ANG_Z].value_;
    publ_.publish(msg);
  }
}


/** TeleopPolicy stop function redefinition.
 *
 * Send a twist levels message with null twist for every DOF.
 */
void auv_teleoperation::TwistPolicy::stop()
{
  ROS_INFO_STREAM("Sending null command on twist policy stop...");
  control_common::TwistLevelsStampedPtr msg(new control_common::TwistLevelsStamped());
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = frame_id_;
  msg->twist.linear.x = 0.0;
  msg->twist.linear.y = 0.0;
  msg->twist.linear.z = 0.0;
  msg->twist.angular.x = 0.0;
  msg->twist.angular.y = 0.0;
  msg->twist.angular.z = 0.0;
  publ_.publish(msg);
}

