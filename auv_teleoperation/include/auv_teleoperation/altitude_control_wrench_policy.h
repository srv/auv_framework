#ifndef ALTITUDE_CONTROL_WRENCH_POLICY_H
#define ALTITUDE_CONTROL_WRENCH_POLICY_H

#include <ros/ros.h>
#include "auv_teleoperation/teleoperation_policy.h"

namespace auv_teleoperation
{

/** 
 * @class AltitudeControlWrenchPolicy
 * @brief Altitude control/wrench teleoperation policy class.
 *
 * Joystick commands are translated to wrench levels
 * and published on the /wrench_request topic.
 * When this policy is selected, the altitude
 * controller is enabled via service calls.
 * So the joystick controls wrench in x, y and RPY directly,
 * z is controlled via altitude_controller.
 *
 * @par Published topics
 *
 *   - @b wrench_request (geometry_msgs/WrenchStamped)
 *   wrench levels with the current time stamp.
 *   - @b altitude_request (std_msgs/Float32)
 *   altitude request for the altitude controller
 *
 * @par Parameters
 *
 * - @b ~frame_id frame name for published messages
 */
class AltitudeControlWrenchPolicy : public TeleoperationPolicy
{
public:
  /** Constructor passing through the node handles
  *
  * @param n node namespace handle
  * @param p private namespace handle
  * @return
  */
  AltitudeControlWrenchPolicy(
      const ros::NodeHandle& n, const ros::NodeHandle& p);

  virtual void start();
  virtual void pause();
  virtual void stop();

protected:
  /**
   * Translates requested DOF values to wrench and sends a 
   * geometry_msgs::WrenchStamped message.
   * Also the setpoint for altitude is updated sending a message
   * on corresponding topic.
   */
  virtual void updateDOFs(const ros::Time& stamp);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  std::string frame_id_;
  ros::Publisher wrench_pub_;
  ros::Publisher altitude_request_pub_;

};

} // namespace

#endif // ALTITUDE_CONTROL_WRENCH_POLICY_H
