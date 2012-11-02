/**
 * @file altitude_control_wrench_policy.h
 * @brief Altitude control wrench teleoperation policy.
 * @author Stephan Wirth
 * @date 2012-10-16
 */

#ifndef ALTITUDE_CONTROL_WRENCH_POLICY_H
#define ALTITUDE_CONTROL_WRENCH_POLICY_H

#include <ros/ros.h>
#include "fugu_teleoperation/teleop_policy.h"

namespace fugu_teleoperation
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
 * Several button actions are implemented:
 *   - increase force/torque offset for each DOF
 *   - decrease force/torque offset for each DOF
 *   - reset force/torque offset for each DOF
 *   - set all DOFs to null state  (without resetting offsets)
 *   - increase/decrease the setpoint for altitude

 * @par Published topics
 *
 *   - @b wrench_request (geometry_msgs/WrenchStamped)
 *   wrench levels with the current time stamp.
 *   - @b altitude_request (std_msgs/Float32)
 *   altitude request for the altitude controller
 *
 * @par Parameters
 *
 * For each degree of freedom (linear_x, linear_y, altitude, angular_x,
 * angular_y, angular_z):
 * - @b ~[dof]_axis joystick axis number
 * - @b ~[dof]_factor joystick axis value multiplier
 * - @b ~[dof]_positive_offset_button increase offset button number
 * - @b ~[dof]_negative_offset_button decrease offset button number
 * - @b ~[dof]_offset_step stepping increment
 * - @b ~[dof]_reset_button reset button number
 *
 * - @b ~pause_button set all forces and torques to null (offsets are preserved, altitude is not touched)
 *
 * - @b ~frame_id frame name for published messages
 */
class AltitudeControlWrenchPolicy : public TeleopPolicy
{
public:
  /** Constructor passing through the node handles
  *
  * The private policy namespace is "altitude_control_wrench_policy" 
  * relative to p's namespace.
  * The policy namespace for publications is n's namespace.
  *
  * @param n node namespace handle
  * @param p private namespace handle
  * @return
  */
  AltitudeControlWrenchPolicy(const ros::NodeHandle& n, const ros::NodeHandle& p);
  void init();
  void update(const JoyState& j); //!< joystick response
  void start();
  void stop();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  ros::Publisher wrench_pub_;
  ros::Publisher altitude_request_pub_;

  std::string frame_id_;

  enum DOFS {
    LINEAR_X, LINEAR_Y, ALTITUDE, ANGULAR_X, ANGULAR_Y, ANGULAR_Z};
  static const int NUM_DOFS = 6;

  /**
   * Button and axis IDs for one degree of freedom
   * together with their stepping & factor settings.
   */
  struct DOFMapping
  {
    int axis_; 
    int positive_offset_button_;
    int negative_offset_button_;
    int reset_button_;
    double factor_;
    double offset_step_;
  };

  DOFMapping dof_map_[NUM_DOFS];

  struct DOFState
  {
    float offset_;
    float value_;
  };

  DOFState dof_state_[NUM_DOFS];

  int pause_button_;

  void initDOFStates();
  void initParams();
  void advertiseTopics();

  /** Update DOFState from joystick state.
  *
  * @param d DOF state.
  * @param m DOF mapping.
  * @param j joystick state
  * @return whether the DOF state is modified by the joystick state.
  */
  bool updateDOFState(DOFState& d, const DOFMapping& m,
                      const fugu_teleoperation::JoyState& j);

};

} // namespace

#endif // ALTITUDE_CONTROL_WRENCH_POLICY_H
