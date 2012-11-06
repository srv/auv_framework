#ifndef WRENCH_POLICY_H
#define WRENCH_POLICY_H

#include <ros/ros.h>
#include "auv_teleoperation/teleoperation_policy.h"

namespace auv_teleoperation
{

/** Wrench teleoperation policy class.
 *
 * The wrench policy responds to joystick events
 * controlling the wrench levels on each DOF.
 */
class WrenchPolicy : public TeleoperationPolicy
{
public:

  /**
   * Constructs a wrench policy with given node handles.
   * The handle nh_priv will be used to get all parameters,
   * the hande nh will be used to advertise topics.
   */
  WrenchPolicy(const ros::NodeHandle& nh, const ros::NodeHandle& nh_priv);

  /**
   * Sends a zero wrench
   */
  virtual void start();

  /**
   * Sends a zero wrench
   */
  virtual void pause();

  /**
   * Sends a zero wrench
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

#endif // WRENCH_POLICY_H
