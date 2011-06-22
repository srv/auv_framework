/** @file
 *
 * @brief Joystick controller for the fugu-c robot.
 * Joystick commands are translated to motor speeds and published on the corresponding topic.
 * Several button actions are implemented:
 *   - +/- linear speed offset for each motor pair
 *   - +/- angular speed offset for each motor pair
 *   - stop motor pair
 *   - stop all motors
 *
 * @par Parameters
 *
 * - @b ~fw_lin_delta forward motor pair linear speed axis number
 * - @b ~fw_lin_poffs forward motor pair increase linear speed offset button number
 * - @b ~fw_lin_moffs forward motor pair decrease linear speed offset button number
 * - @b ~fw_ang_delta forward motor pair angular speed axis number
 * - @b ~fw_ang_poffs forward motor pair increase angular speed offset button number
 * - @b ~fw_ang_moffs forward motor pair decrease angular speed offset button number
 * - @b ~fw_stop_bttn forward motor pair stop button number
 *
 * - @b ~dw_lin_delta downward motor pair linear speed axis number
 * - @b ~dw_lin_poffs downward motor pair increase linear speed offset button number
 * - @b ~dw_lin_moffs downward motor pair decrease linear speed offset button number
 * - @b ~dw_ang_delta downward motor pair angular speed axis number
 * - @b ~dw_ang_poffs downward motor pair increase angular speed offset button number
 * - @b ~dw_ang_moffs downward motor pair decrease angular speed offset button number
 * - @b ~dw_stop_bttn downward motor pair stop button number
 *
 * - @b ~pause_bttn pause all motors button number (offsets are preserved)
 *
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
 */

#include <ros/ros.h>
#include <joy/Joy.h>
#include <control_common/control_types.h>
#include <fugu_teleoperation/joy_state.h>

class FuguTeleopJoyNode
{
public:
  FuguTeleopJoyNode();
  void initParams();
  void advertiseTopics();
  void subscribeTopics();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  ros::Publisher publ_;
  ros::Subscriber joy_subs_;

  std::string frame_id_;

  fugu_teleoperation::JoyState joy_state_;

  enum DOFS {LIN_X, LIN_Y, LIN_Z, ANG_X, ANG_Y, ANG_Z};
  static const int NUM_DOFS = 6;

  struct DOFMapping
  {
    int incrm_axis_;
    int poffs_bttn_;
    int noffs_bttn_;
    int reset_bttn_;
    double factor_;
    double step_;
  };

  DOFMapping dof_map_[NUM_DOFS];

  struct DOFState
  {
    float offst_;
    float incrm_;
    float value_;
  };

  DOFState dof_state_[NUM_DOFS];

  bool updateDOFState(DOFState& s,
                      const DOFMapping& m, const fugu_teleoperation::JoyState& j);

  void joyCallback(const joy::Joy::ConstPtr& joy);

};


FuguTeleopJoyNode::FuguTeleopJoyNode()
: priv_("~")
{}

void FuguTeleopJoyNode::initParams()
{
  std::string dof_names[NUM_DOFS];
  dof_names[LIN_X] = "lin_x";
  dof_names[LIN_Y] = "lin_y";
  dof_names[LIN_Z] = "lin_z";
  dof_names[ANG_X] = "ang_x";
  dof_names[ANG_Y] = "ang_y";
  dof_names[ANG_Z] = "ang_z";
  for (int i=0; i<NUM_DOFS; i++)
  {
    // Set mapping from parameter server
    std::string dof_name = dof_names[i];
    ROS_INFO_STREAM("Setting mapping for " << dof_name << "...");
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
    ROS_DEBUG_STREAM(dof_name+" reset button   : " << dof_map_[i].reset_bttn_);
    // Initialize DOF state
    dof_state_[i].incrm_ = 0.0;
    dof_state_[i].offst_ = 0.0;
    dof_state_[i].value_ = 0.0;
  }

  priv_.param("frame_id",frame_id_,std::string());
//  priv_.param("pause_bttn",pause_ctrl.id,-1);
//  ROS_INFO_STREAM("Pause button set to " << pause_ctrl.id);
}

void FuguTeleopJoyNode::advertiseTopics()
{
  publ_ = nh_.advertise<control_common::WrenchLevelsStamped>("wrench_levels", 10);
}

void FuguTeleopJoyNode::subscribeTopics()
{
  joy_subs_ = nh_.subscribe<joy::Joy>("joy", 10,
                                      &FuguTeleopJoyNode::joyCallback, this);
}

bool FuguTeleopJoyNode::updateDOFState(DOFState& s,
                                       const DOFMapping& m,
                                       const fugu_teleoperation::JoyState& j)
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

void FuguTeleopJoyNode::joyCallback(const joy::Joy::ConstPtr& joy)
{
  bool updated = false;
  joy_state_.update(joy);
  for (int i=0; i<NUM_DOFS; i++)
    if ( updateDOFState(dof_state_[i], dof_map_[i], joy_state_) )
      updated = true;
  if ( updated )
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fugu_teleop_joy_node");
  FuguTeleopJoyNode teleop_fugu;
  teleop_fugu.initParams();
  teleop_fugu.advertiseTopics();
  teleop_fugu.subscribeTopics();
  ros::spin();
}
