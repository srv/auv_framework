/** @file joy_dof_mapping.h
 * @brief Mapping from joystick state to states of degrees of freedom
 */
#ifndef JOY_DOF_MAPPING_H
#define JOY_DOF_MAPPING_H

#include "auv_teleoperation/joy_state.h"
#include "auv_teleoperation/dof_state.h"

#include <ros/ros.h>

namespace auv_teleoperation
{

/**
 * Exception for missing or misconfigured parameters
 */
class ParameterError : public std::runtime_error
{
public:
  /**
   * Constructor forwarding error string to base constructor.
   */
  ParameterError(const std::string& what) :
    std::runtime_error(what)
  {}
};

/** 
 * @class JoyDOFMapping
 * @brief Maps joystick axes and buttons to states of degrees of freedom.
 *
 * The mapping from joystick axes and buttons to degrees of freedom can
 * be controlled by ROS parameters.
 *
 */
class JoyDOFMapping
{
public:

  /**
   * Updates all DOF states from the joystick state. The internally stored
   * mapping is used to translate joystick inputs to DOF state changes.
   * @param joy_state input joy state
   * @param dof_states the DOF states to update from the joystick input
   * @return true if any DOF has been updated, false otherwise
   */
  bool update(const JoyState& joy_state, std::vector<DOFState>& dof_states);

  /**
   * Constructs a new mapping from joystick to degrees of freedom
   * retrieving the following parameters from the parameter server
   * using given node handle. All parameters have to have the same
   * length equal the number of degrees of freedom that have to be
   * controlled by the joystick. Negative indices mean that no button
   * or axis has to be mapped for the corresponding action.
   * - joy_axes: indices for joy axes that control a DOF
   * - positive_offset_buttons: buttons to increase the offset of a DOF
   * - negative_offset_buttons: buttons to decrease the offset of a DOF
   * - offset_reset_buttons: buttons to reset the offset of a DOF
   * - joy_axes_factors: multiplicator for the joy value for each DOF
   * - offset_steps: stepping size for offsets for each DOF
   *
   * @param num_dofs Number of degrees of freedom
   * @param nh node handle to read the parameters from. If necessary
   *        parameters are not found, an exception is thrown.
   */
  void init(int num_dofs, const ros::NodeHandle& nh);

private:

  /**
   * Mapping of one axis and offset buttons to one DOF
   */
  struct JoyDOFSetting
  {
    int joy_axis;
    int negative_offset_button;
    int positive_offset_button;
    int offset_reset_button;
    float axis_factor;
    float offset_step;
  };

  std::vector<JoyDOFSetting> joy_dof_settings_;

};

} // namespace

#endif // JOY_DOF_MAPPING_H
