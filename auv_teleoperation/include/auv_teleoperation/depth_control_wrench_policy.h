/**
 * @file depth_control_wrench_policy.h
 * @brief Depth control wrench teleoperation policy.
 * @author Stephan Wirth
 * @date 2012-10-16
 */

#ifndef DEPTH_CONTROL_WRENCH_POLICY_H
#define DEPTH_CONTROL_WRENCH_POLICY_H

#include <ros/ros.h>
#include "auv_teleoperation/teleoperation_policy.h"

namespace auv_teleoperation
{

/** 
 * @class DepthControlWrenchPolicy
 * @brief Depth control/wrench teleoperation policy class.
 *
 * Joystick commands are translated to wrench levels
 * and published on the /wrench_request topic.
 * When this policy is selected, the depth
 * controller is enabled via service calls.
 * So the joystick controls wrench in x, y and RPY directly,
 * z is controlled via depth_controller.
 *
 * @par Published topics
 *
 *   - @b wrench_request (geometry_msgs/WrenchStamped)
 *   wrench levels with the current time stamp.
 *   - @b depth_request (std_msgs/Float32)
 *   depth request for the depth controller
 *
 * @par Parameters
 * - @b ~frame_id frame name for published messages
 */
class DepthControlWrenchPolicy : public TeleoperationPolicy
{
public:
  /** Constructor passing through the node handles
  *
  * @param n node namespace handle
  * @param p private namespace handle
  * @return
  */
  DepthControlWrenchPolicy(
      const ros::NodeHandle& n, const ros::NodeHandle& p);

  virtual void start();
  virtual void pause();
  virtual void stop();

protected:
  /**
   * Translates requested DOF values to wrench and sends a 
   * geometry_msgs::WrenchStamped message.
   * Also the setpoint for depth is updated sending a message
   * on corresponding topic.
   */
  virtual void updateDOFs(const ros::Time& stamp);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  std::string frame_id_;
  ros::Publisher wrench_pub_;
  ros::Publisher depth_request_pub_;

};

} // namespace

#endif // DEPTH_CONTROL_WRENCH_POLICY_H
