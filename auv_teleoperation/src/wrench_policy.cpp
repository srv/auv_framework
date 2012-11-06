#include <geometry_msgs/WrenchStamped.h>
#include "auv_teleoperation/wrench_policy.h"

auv_teleoperation::WrenchPolicy::WrenchPolicy(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_priv) :
  TeleoperationPolicy(6, nh_priv), nh_(nh), nh_priv_(nh_priv)
{
  nh_priv_.param("frame_id", frame_id_, std::string("base_link"));
  ROS_DEBUG_STREAM("Frame id set to " << frame_id_);

  ROS_INFO_STREAM("Advertising teleoperation wrench as " <<
      nh_priv_.resolveName("wrench"));
  pub_ = nh_priv_.advertise<geometry_msgs::WrenchStamped>(
      "wrench", 1);
}

void auv_teleoperation::WrenchPolicy::start()
{
  ROS_INFO_STREAM("Starting wrench policy...");
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  pub_.publish(msg);
}

void auv_teleoperation::WrenchPolicy::pause()
{
  ROS_INFO_STREAM("Sending null wrench on wrench policy pause...");
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  pub_.publish(msg);
}

void auv_teleoperation::WrenchPolicy::stop()
{
  ROS_INFO_STREAM("Sending null command on wrench policy stop...");
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  pub_.publish(msg);
}

void auv_teleoperation::WrenchPolicy::updateDOFs(const ros::Time& stamp)
{
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.wrench.force.x = dof_states_[0].getValue();
  msg.wrench.force.y = dof_states_[1].getValue();
  msg.wrench.force.z = dof_states_[2].getValue();
  msg.wrench.torque.x = dof_states_[3].getValue();
  msg.wrench.torque.y = dof_states_[4].getValue();
  msg.wrench.torque.z = dof_states_[5].getValue();
  pub_.publish(msg);
}

