#include <geometry_msgs/TwistStamped.h>
#include "auv_teleoperation/twist_policy.h"

auv_teleoperation::TwistPolicy::TwistPolicy(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_priv) :
  TeleoperationPolicy(6, nh_priv), nh_(nh), nh_priv_(nh_priv)
{
  nh_priv_.param("frame_id", frame_id_, std::string("base_link"));
  ROS_DEBUG_STREAM("Frame id set to " << frame_id_);

  ROS_INFO_STREAM("Advertising teleoperation twist as " <<
      nh_priv_.resolveName("twist"));
  pub_ = nh_priv_.advertise<geometry_msgs::TwistStamped>(
      "twist", 1);
}

void auv_teleoperation::TwistPolicy::start()
{
  ROS_INFO_STREAM("Starting twist policy...");
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  pub_.publish(msg);
}

void auv_teleoperation::TwistPolicy::pause()
{
  ROS_INFO_STREAM("Sending null twist on twist policy pause...");
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  pub_.publish(msg);
}

void auv_teleoperation::TwistPolicy::stop()
{
  ROS_INFO_STREAM("Sending null command on twist policy stop...");
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  pub_.publish(msg);
}

void auv_teleoperation::TwistPolicy::updateDOFs(const ros::Time& stamp)
{
  geometry_msgs::TwistStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.twist.linear.x = dof_states_[0].getValue();
  msg.twist.linear.y = dof_states_[1].getValue();
  msg.twist.linear.z = dof_states_[2].getValue();
  msg.twist.angular.x = dof_states_[3].getValue();
  msg.twist.angular.y = dof_states_[4].getValue();
  msg.twist.angular.z = dof_states_[5].getValue();
  pub_.publish(msg);
}

