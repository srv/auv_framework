#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "auv_teleoperation/joy_state.h"
#include "auv_teleoperation/motor_policy.h"
#include "auv_teleoperation/wrench_policy.h"
#include "auv_teleoperation/altitude_control_wrench_policy.h"
#include "auv_teleoperation/depth_control_wrench_policy.h"
#include "auv_teleoperation/twist_policy.h"

class TeleoperationJoyNode
{
public:
  TeleoperationJoyNode();

private:

  void changePolicy();
  void joyCallback(const sensor_msgs::JoyConstPtr& joy);

  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  ros::Subscriber joy_sub_;

  typedef boost::shared_ptr<auv_teleoperation::TeleoperationPolicy> TeleoperationPolicyPtr;
  std::vector<TeleoperationPolicyPtr> policies_;
  std::vector<TeleoperationPolicyPtr>::iterator current_policy_;

  int policy_button_;
  int pause_button_;
  auv_teleoperation::JoyState joy_state_;
};

TeleoperationJoyNode::TeleoperationJoyNode()
: priv_("~")
{
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
    &TeleoperationJoyNode::joyCallback, this);
  ROS_INFO_STREAM("Setting general teleoperation parameters...");
  priv_.param("policy_button", policy_button_, -1);
  ROS_INFO_STREAM("Policy button set to : " << policy_button_ );
  priv_.param("pause_button", pause_button_, -1);
  ROS_INFO_STREAM("Pause button set to : " << pause_button_ );

  ROS_INFO_STREAM("Loading policies...");
  // we push down the private namespace into each policy to read their
  // parameters
  policies_.push_back(TeleoperationPolicyPtr(new auv_teleoperation::MotorPolicy(
        nh_, ros::NodeHandle(priv_, "motor_policy"))));
  policies_.push_back(TeleoperationPolicyPtr(new auv_teleoperation::WrenchPolicy(
        nh_, ros::NodeHandle(priv_, "wrench_policy"))));
  policies_.push_back(TeleoperationPolicyPtr(new auv_teleoperation::AltitudeControlWrenchPolicy(
        nh_, ros::NodeHandle(priv_, "altitude_control_wrench_policy"))));
  policies_.push_back(TeleoperationPolicyPtr(new auv_teleoperation::DepthControlWrenchPolicy(
        nh_, ros::NodeHandle(priv_, "depth_control_wrench_policy"))));
  policies_.push_back(TeleoperationPolicyPtr(new auv_teleoperation::TwistPolicy(
        nh_, ros::NodeHandle(priv_, "twist_policy"))));
  current_policy_ = policies_.begin();
  (*current_policy_)->start();
}

void TeleoperationJoyNode::changePolicy()
{
  ROS_INFO_STREAM("Switching to next policy...");
  (*current_policy_)->stop();
  ++current_policy_;
  if (current_policy_ == policies_.end())
    current_policy_ = policies_.begin();
  (*current_policy_)->start();
}

void TeleoperationJoyNode::joyCallback(const sensor_msgs::JoyConstPtr& joy)
{
  joy_state_.update(joy);
  if (joy_state_.buttonPressed(policy_button_))
    changePolicy();
  else if (joy_state_.buttonPressed(pause_button_))
    (*current_policy_)->pause();
  else
    (*current_policy_)->update(joy_state_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleoperation_joy_node");
  TeleoperationJoyNode teleoperation_node;
  ros::spin();
}

