#ifndef DOF_STATE_H
#define DOF_STATE_H

namespace auv_teleoperation
{

/**
 * State of a degree of freedom.
 */
struct DOFState
{
  float offset;
  float value;
  bool updated; // flag to mark if state changed in last update
  float getValue() const 
  {
    return offset + value;
  }
};

}
#endif
