/**
 * @file wrench_policy.h
 * @brief Wrench teleoperation policy presentation.
 * @author Joan Pau Beltran
 * @date 2011-06-29
 */

#ifndef WRENCH_POLICY_H
#define WRENCH_POLICY_H

#include <ros/ros.h>
#include "fugu_teleoperation/teleop_policy.h"

namespace fugu_teleoperation
{

/** Wrench teleoperation policy class.
 *
 * The wrench policy responds to joystick events controlling the wrench levels
 * on the surge, heave, roll and yaw DOF.
 */
class WrenchPolicy : public TeleopPolicy
{
public:
  WrenchPolicy(const ros::NodeHandle& n, const ros::NodeHandle& p);
  void init();
  void update(const JoyState& s); //!< joystick response
  void stop();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_;
  ros::Publisher publ_;

  std::string frame_id_;

  enum DOFS {LIN_X, LIN_Y, LIN_Z, ANG_X, ANG_Y, ANG_Z};
  static const int NUM_DOFS = 6;

  struct DOFMapping
  {
    int incrm_axis_;
    int poffs_bttn_;
    int noffs_bttn_;
    int reset_bttn_;
    double factor_;
    double step_;
  };

  DOFMapping dof_map_[NUM_DOFS];

  struct DOFState
  {
    float offst_;
    float incrm_;
    float value_;
  };

  DOFState dof_state_[NUM_DOFS];

  int pause_bttn_;

  void initDOFStates();
  void initParams();
  void advertiseTopics();

  bool updateDOFState(DOFState& s, const DOFMapping& m,
                      const fugu_teleoperation::JoyState& j);

};

} // namespace

#endif // WRENCH_POLICY_H
