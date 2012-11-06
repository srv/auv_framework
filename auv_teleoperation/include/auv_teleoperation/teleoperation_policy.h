#ifndef TELEOPERATION_POLICY_H
#define TELEOPERATION_POLICY_H

#include "auv_teleoperation/joy_state.h"
#include "auv_teleoperation/joy_dof_mapping.h"

namespace auv_teleoperation
{

/** Teleoperation policy interface.
 *
 * Teleoperation policies provide different responses to joystick states.
 *
 */
class TeleoperationPolicy
{
public:

  virtual ~TeleoperationPolicy() {};

  /**
   * Updates all DOFs that are teleoperated.
   */
  void update(const JoyState& joy_state)
  {
    if (joy_dof_mapping_.update(joy_state, dof_states_))
    {
      updateDOFs(joy_state.stamp());
    }
  }

  /**
   * Resets all DOF states to zero.
   */
  void resetDOFStates()
  {
    for (size_t i = 0; i < dof_states_.size(); ++i)
    {
      dof_states_[i].offset = 0.0;
      dof_states_[i].value = 0.0;
    }
  }

  /**
   * To be called on selecting the policy
   */
  virtual void start() = 0;

  /**
   * To be called on pausing the policy
   */
  virtual void pause() = 0;

  /**
   * To be called on switching away from this policy
   */
  virtual void stop() = 0;

protected:

  /**
   * Initialize the teleoperation policy.
   * @param num_dofs number of degrees of freedom the DOF
   *        mapping has to have
   * @param param_nh Node handle to read the joystick DOF 
   *        mapping parameters from
   */
  TeleoperationPolicy(int num_dofs, const ros::NodeHandle& param_nh)
  {
    joy_dof_mapping_.init(num_dofs, param_nh);
    dof_states_.resize(num_dofs);
    resetDOFStates();
  };

  /**
   * To be implemented by derived classes. Do the necessary things,
   * i.e. send messages to control the DOFs.
   * Will be called if a DOF is changed by the joystick.
   * @param stamp time stamp of the last joy message
   */
  virtual void updateDOFs(const ros::Time& stamp) = 0;

  /// current state of the DOFs, continuously updated by joystick inputs
  std::vector<DOFState> dof_states_;

private:

  int pause_button_;
  JoyDOFMapping joy_dof_mapping_;

};

} // namespace

#endif // TELEOPERATION_POLICY_H
