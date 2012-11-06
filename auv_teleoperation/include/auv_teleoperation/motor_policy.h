#ifndef MOTOR_POLICY_H
#define MOTOR_POLICY_H

#include <ros/ros.h>
#include <Eigen/Core>
#include "auv_teleoperation/teleoperation_policy.h"

namespace auv_teleoperation
{

/** Motor teleoperation policy class.
 *
 * The motor policy responds to joystick events
 * controlling the motor levels on each motor.
 */
class MotorPolicy : public TeleoperationPolicy
{
public:
  /**
   * Constructs a motor policy with given node handles.
   * The handle nh_priv will be used to get all parameters,
   * the hande nh will be used to advertise topics.
   */
  MotorPolicy(ros::NodeHandle& nh, const ros::NodeHandle& nh_priv);

  /**
   * Sends motor levels set to zero
   */
  virtual void start();

  /**
   * Sends motor levels set to zero
   */
  virtual void pause();

  /**
   * Sends motor levels set to zero
   */
  virtual void stop();

protected:
  /**
   * translates requested DOF values to motor levels using the
   * axes_to_motors_ matrix and sends an auv_control_msgs::MotorLevels
   * message.
   */
  virtual void updateDOFs(const ros::Time& stamp);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  std::string frame_id_;
  ros::Publisher pub_;
  Eigen::MatrixXd axes_to_motors_;
};

} // namespace

#endif // MOTOR_POLICY_H
