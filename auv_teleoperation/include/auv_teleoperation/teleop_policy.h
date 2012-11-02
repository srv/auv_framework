/** @file teleop_policy.h
 *
 * @brief Teleoperation policy interface.
 *
 */
#ifndef TELEOP_POLICY_H
#define TELEOP_POLICY_H

#include "auv_teleoperation/joy_state.h"

namespace auv_teleoperation
{

/** Teleoperation policy interface.
 *
 * Teleoperation policies provide different responses to joystick states.
 * Usually, concrete policies are derived from this abstract class,
 * implementing the constructor, update() function,
 * and optionally init(), start() and stop() functions.
 *
 * The init() function is supposed to initialize the policy
 * (e.g. set joystick mappping and other parameters, advertise topics).
 *
 * The stop() function is supposed to perform the needed cleanup or reset.
 *
 */
class TeleopPolicy
{
public:
  virtual void init() {}; //!< initialize the policy
  virtual void update(const JoyState& j) = 0; //!< joystick response
  virtual void start() {};
  virtual void stop() {};
  virtual ~TeleopPolicy() {};
};

} // namespace

#endif // TELEOP_POLICY_H
