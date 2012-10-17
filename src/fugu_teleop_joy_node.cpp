/** @file
 *
 * @brief Joystick controller for the fugu robot series.
 * 
 * Joystick commands are translated according to different policies that can
 * be switched on line with a joystick button.
 * 
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "fugu_teleoperation/joy_state.h"
#include "fugu_teleoperation/motor_policy.h"
#include "fugu_teleoperation/wrench_policy.h"
#include "fugu_teleoperation/altitude_control_wrench_policy.h"
#include "fugu_teleoperation/twist_policy.h"


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
  void joyCallback(const sensor_msgs::JoyConstPtr& joy);
};

FuguTeleopJoyNode::FuguTeleopJoyNode()
: priv_("~")
{}

void FuguTeleopJoyNode::initParams()
{
  ROS_INFO_STREAM("Setting general teleoperation parameters...");
  priv_.param("policy_bttn",policy_bttn_, -1);
  ROS_DEBUG_STREAM("Policy button set to : " << policy_bttn_ );
}

void FuguTeleopJoyNode::loadPolicies()
{
  ROS_INFO_STREAM("Loading policies...");
  TeleopPolicyPtr motor_policy_ptr_( new fugu_teleoperation::MotorPolicy(nh_,priv_) );
  TeleopPolicyPtr wrench_policy_ptr_( new fugu_teleoperation::WrenchPolicy(nh_,priv_) );
  TeleopPolicyPtr altitude_control_wrench_policy_ptr_( new fugu_teleoperation::AltitudeControlWrenchPolicy(nh_,priv_) );
  TeleopPolicyPtr twist_policy_ptr_( new fugu_teleoperation::TwistPolicy(nh_,priv_) );
  policies_.push_back(motor_policy_ptr_);
  policies_.push_back(wrench_policy_ptr_);
  policies_.push_back(altitude_control_wrench_policy_ptr_);
  policies_.push_back(twist_policy_ptr_);
  for (std::vector<TeleopPolicyPtr>::iterator p=policies_.begin();
       p<policies_.end();
       p++)
    (*p)->init();
  current_policy_ = policies_.begin();
  (*current_policy_)->start();
}

void FuguTeleopJoyNode::changePolicy()
{
  ROS_INFO_STREAM("Switching to next policy...");
  (*current_policy_)->stop();
  if (++current_policy_ == policies_.end())
    current_policy_ = policies_.begin();
  (*current_policy_)->start();
}

void FuguTeleopJoyNode::subscribeTopics()
{
  joy_subs_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
                                              &FuguTeleopJoyNode::joyCallback, this);
}

void FuguTeleopJoyNode::joyCallback(const sensor_msgs::JoyConstPtr& joy)
{
  joy_state_.update(joy);
  if (joy_state_.buttonPressed(policy_bttn_))
    changePolicy();
  (*current_policy_)->update(joy_state_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fugu_teleop_joy_node");
  FuguTeleopJoyNode teleop_fugu;
  teleop_fugu.initParams();
  teleop_fugu.loadPolicies();
  teleop_fugu.subscribeTopics();
  ros::spin();
}
