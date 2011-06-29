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
#include "fugu_teleoperation/joy_state.h"
#include "fugu_teleoperation/wrench_policy.h"


class FuguTeleopJoyNode
{
public:
  FuguTeleopJoyNode();
  void initParams();
  void loadPolicies();
  void subscribeTopics();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  ros::Publisher publ_;
  ros::Subscriber joy_subs_;

  typedef boost::shared_ptr<fugu_teleoperation::TeleopPolicy> TeleopPolicyPtr;
  std::vector<TeleopPolicyPtr> policies_;
  std::vector<TeleopPolicyPtr>::iterator current_policy_;
  void changePolicy();

  int policy_bttn_;
  fugu_teleoperation::JoyState joy_state_;
  void joyCallback(const joy::Joy::ConstPtr& joy);
};

FuguTeleopJoyNode::FuguTeleopJoyNode()
: priv_("~")
{}

void FuguTeleopJoyNode::initParams()
{
  priv_.param("policy_bttn",policy_bttn_, -1);
  ROS_DEBUG_STREAM("Policy button set to : " << policy_bttn_ );
}

void FuguTeleopJoyNode::loadPolicies()
{
  TeleopPolicyPtr wrench_policy_ptr_( new fugu_teleoperation::WrenchPolicy(nh_,priv_) );
  policies_.push_back(wrench_policy_ptr_);
  current_policy_ = policies_.begin();
  (*current_policy_)->init();
}

void FuguTeleopJoyNode::changePolicy()
{
  (*current_policy_)->stop();
  if (++current_policy_ == policies_.end())
    current_policy_ = policies_.begin();
  (*current_policy_)->init();
}

void FuguTeleopJoyNode::subscribeTopics()
{
  joy_subs_ = nh_.subscribe<joy::Joy>("joy", 10,
                                      &FuguTeleopJoyNode::joyCallback, this);
}

void FuguTeleopJoyNode::joyCallback(const joy::Joy::ConstPtr& joy)
{
  joy_state_.update(joy);
  if (joy_state_.buttonPressed(policy_bttn_))
  {
    changePolicy();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fugu_teleop_joy_node");
  FuguTeleopJoyNode teleop_fugu;
  teleop_fugu.initParams();
  teleop_fugu.subscribeTopics();
  ros::spin();
}
