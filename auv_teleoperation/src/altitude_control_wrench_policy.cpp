#include <std_msgs/Float32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <auv_control_msgs/EnableControl.h>

#include "auv_teleoperation/altitude_control_wrench_policy.h"

auv_teleoperation::AltitudeControlWrenchPolicy::AltitudeControlWrenchPolicy(
    const ros::NodeHandle& nh, const ros::NodeHandle& nh_priv) :
  TeleoperationPolicy(6, nh_priv), nh_(nh), nh_priv_(nh_priv)
{
  nh_priv_.param("frame_id",frame_id_,std::string("base_link"));
  ROS_DEBUG_STREAM("Frame id set to " << frame_id_);
  ROS_INFO_STREAM("Advertising teleoperation wrench as " <<
      nh_priv_.resolveName("wrench"));
  wrench_pub_ = nh_priv_.advertise<geometry_msgs::WrenchStamped>(
      "wrench", 1);
  bool latched = true;
  ROS_INFO_STREAM("Advertising altitude request as " <<
      nh_priv.resolveName("altitude_request"));
  altitude_request_pub_ = nh_priv_.advertise<std_msgs::Float32>(
      "altitude_request", 1, latched);
}

void auv_teleoperation::AltitudeControlWrenchPolicy::start()
{
  ROS_INFO_STREAM("Starting altitude control wrench policy...");
  resetDOFStates();
  ros::ServiceClient client = 
    nh_.serviceClient<auv_control_msgs::EnableControl>(
        "enable_altitude_control");
  auv_control_msgs::EnableControl control_service;
  control_service.request.enable = true;
  if (client.call(control_service))
  {
    if (control_service.response.enabled)
    {
      ROS_INFO("Altitude control enabled.");
      dof_states_[2].offset = 
        control_service.response.current_setpoint;
    }
    else
    {
      ROS_ERROR("Failed to enable altitude control!");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service %s", 
        nh_.resolveName("enable_altitude_control").c_str());
  }
  updateDOFs(ros::Time::now());
}

void auv_teleoperation::AltitudeControlWrenchPolicy::pause()
{
  ROS_INFO_STREAM(
      "Sending null wrench on altitude control policy pause...");
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = frame_id_;
  wrench_pub_.publish(msg);
}

void auv_teleoperation::AltitudeControlWrenchPolicy::stop()
{
  resetDOFStates();
  updateDOFs(ros::Time::now());
  ros::ServiceClient client = 
    nh_.serviceClient<auv_control_msgs::EnableControl>(
        "enable_altitude_control");
  auv_control_msgs::EnableControl control_service;
  control_service.request.enable = false;
  if (client.call(control_service))
  {
    if (!control_service.response.enabled)
    {
      ROS_INFO("Altitude control disabled.");
    }
    else
    {
      ROS_ERROR("Failed to disable altitude control!");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service %s", 
        nh_.resolveName("enable_altitude_control").c_str());
  }
}

void auv_teleoperation::AltitudeControlWrenchPolicy::updateDOFs(
     const ros::Time& stamp)
{
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id_;
  msg.wrench.force.x = dof_states_[0].getValue();
  msg.wrench.force.y = dof_states_[1].getValue();
  msg.wrench.torque.x = dof_states_[3].getValue();
  msg.wrench.torque.y = dof_states_[4].getValue();
  msg.wrench.torque.z = dof_states_[5].getValue();
  wrench_pub_.publish(msg);

  if (dof_states_[2].updated)
  {
    std_msgs::Float32 altitude_request_msg;
    altitude_request_msg.data = dof_states_[2].getValue();
    ROS_INFO("Sending altitude request: %f", altitude_request_msg.data);
    altitude_request_pub_.publish(altitude_request_msg);
  }
}

