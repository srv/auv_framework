#ifndef TWIST_POLICY_H
#define TWIST_POLICY_H

#include <ros/ros.h>
#include "auv_teleoperation/teleoperation_policy.h"

namespace auv_teleoperation
{

/** Twist teleoperation policy class.
 *
 * The twist policy responds to joystick events
 * controlling the twist levels on each DOF.
 */
class TwistPolicy : public TeleoperationPolicy
{
public:
  /**
   * Constructs a wrench policy with given node handles.
   * The handle nh_priv will be used to get all parameters,
   * the hande nh will be used to advertise topics.
   */
  TwistPolicy(const ros::NodeHandle& nh, const ros::NodeHandle& nh_priv);

  /**
   * Sends a zero twist
   */
  virtual void start();

  /**
   * Sends a zero twist
   */
  virtual void pause();

  /**
   * Sends a zero twist
   */
  virtual void stop();

protected:
  /**
   * Puts all dof_values in a twist message with given stamp
   * and sends it.
   */
  virtual void updateDOFs(const ros::Time& stamp);

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  
  std::string frame_id_;
  ros::Publisher pub_;

};

} // namespace

#endif // TWIST_POLICY_H
