/**
 * @file wrench_policy.h
 * @brief Wrench teleoperation policy presentation.
 * @author Joan Pau Beltran
 * @date 2011-06-29
 */

#include "fugu_teleoperation/wrench_policy.h"
#include "control_common/control_types.h"
#include <string>


/** Constructor passing through the node handles
 *
 * The private policy namespace is "wrench_policy" relative to p's namespace.
 * The policy namespace for publications is n's namespace. *
 *
 * @param n node namespace handle
 * @param p private namespace handle
 * @return
 */
fugu_teleoperation::WrenchPolicy::WrenchPolicy(const ros::NodeHandle& n,
                                               const ros::NodeHandle& p)
{
  nh_ = n;
  priv_ = ros::NodeHandle(p,"wrench_policy");
}


/** Initialize DOF states to null state.
 */
void fugu_teleoperation::WrenchPolicy::initDOFStates()
{
  for (int i=0; i<NUM_DOFS; i++)
  {
    dof_state_[i].incrm_ = 0.0;
    dof_state_[i].offst_ = 0.0;
    dof_state_[i].value_ = 0.0;
  }
}


/** Set parameters from parameter server.
 */
void fugu_teleoperation::WrenchPolicy::initParams()
{
  ROS_INFO_STREAM("Setting wrench policy mapping...");
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
    ROS_DEBUG_STREAM("Setting mapping for " << dof_name << "...");
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

  priv_.param("frame_id",frame_id_,std::string("body_fixed"));
  ROS_DEBUG_STREAM("Frame id set to " << frame_id_);

}


/** Advertise "wrench_levels" topic.
 */
void fugu_teleoperation::WrenchPolicy::advertiseTopics()
{
  ROS_INFO_STREAM("Advertising teleoperation wrench levels...");
  publ_ = nh_.advertise<control_common::WrenchLevelsStamped>("wrench_levels", 10);
}


/** TeleopPolicy init function redefinition
 *
 * Initialize parameters from server, initialize DOF states and advertise
 * wrench levels topic.
 */
void fugu_teleoperation::WrenchPolicy::init()
{
  // Set mapping and other parameters from parameter server
  initParams();

  // Initialize DOF states
  initDOFStates();

  // Advertise wrench levels topic
  advertiseTopics();
}

bool fugu_teleoperation::WrenchPolicy::updateDOFState(DOFState& s,
                                                      const DOFMapping& m,
                                                      const JoyState& j)
{
  bool update = false;
  if ( j.buttonPressed(m.reset_bttn_) )
  {
    s.offst_ = 0.0;
    update = true;
  }
  if ( j.buttonPressed(m.noffs_bttn_) )
  {
    s.offst_ -= m.step_;
    update = true;
  }
  if ( j.buttonPressed(m.poffs_bttn_) )
  {
    s.offst_ += m.step_;
    update = true;
  }
  if ( j.axisMoved(m.incrm_axis_) )
  {
    s.incrm_ = m.factor_*j.axisPosition(m.incrm_axis_);
    update = true;
  }
  if (update)
    s.value_ = s.offst_ + s.incrm_;
  return update;
}

void fugu_teleoperation::WrenchPolicy::update(const JoyState& s)
{
  bool updated = false;
  bool pause = s.buttonPressed(pause_bttn_);
  for (int i=0; i<NUM_DOFS; i++)
  {
    if ( updateDOFState(dof_state_[i], dof_map_[i], s) )
      updated = true;
    if ( pause )
      dof_state_[i].value_ = 0.0;
  }

  if ( updated || pause )
  {
    control_common::WrenchLevelsStampedPtr msg(new control_common::WrenchLevelsStamped());
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = frame_id_;
    msg->wrench.force.x = dof_state_[LIN_X].value_;
    msg->wrench.force.y = dof_state_[LIN_Y].value_;
    msg->wrench.force.z = dof_state_[LIN_Z].value_;
    msg->wrench.torque.x = dof_state_[ANG_X].value_;
    msg->wrench.torque.y = dof_state_[ANG_Y].value_;
    msg->wrench.torque.z = dof_state_[ANG_Z].value_;
    publ_.publish(msg);
  }
}

void fugu_teleoperation::WrenchPolicy::stop()
{
  ROS_INFO_STREAM("Sending null command on wrench policy stop...");
  control_common::WrenchLevelsStampedPtr msg(new control_common::WrenchLevelsStamped());
  msg->header.stamp = ros::Time::now();
  msg->header.frame_id = frame_id_;
  msg->wrench.force.x = 0.0;
  msg->wrench.force.y = 0.0;
  msg->wrench.force.z = 0.0;
  msg->wrench.torque.x = 0.0;
  msg->wrench.torque.y = 0.0;
  msg->wrench.torque.z = 0.0;
  publ_.publish(msg);
  publ_.shutdown();
}


