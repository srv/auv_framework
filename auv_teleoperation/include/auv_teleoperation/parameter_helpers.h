#ifndef PARAMETER_HELPERS_H
#define PARAMETER_HELPERS_H

#include <string>
#include <vector>
#include <ros/ros.h>
namespace auv_teleoperation
{
namespace parameter_helpers
{

/**
 * @brief helper for reading uniform parameter lists from the parameter server
 * @param nh node handle for parameter namespace
 * @param name name of the param
 * @param type the list element type of the param to read
 * @return list of values
 */
template <typename T>
std::vector<T> read_param_list(const ros::NodeHandle& nh, const std::string& name, XmlRpc::XmlRpcValue::Type type)
{ 
  XmlRpc::XmlRpcValue list;
  nh.getParam(name, list);
  ROS_ASSERT_MSG(list.getType() == XmlRpc::XmlRpcValue::TypeArray,
      "Parameter %s is not a list!", name.c_str());
  std::vector<T> values(list.size());
  for (size_t i = 0; i < values.size(); ++i) 
  {
    ROS_ASSERT_MSG(list[i].getType() == type, 
        "Elements of parameter %s do not have requested type "
        "(%i instead of %i)", name.c_str(), list[i].getType(),
        type);
    values[i] = static_cast<T>(list[i]);
  }
  return values;
}

} // namespace parameter_helpers
} // namespace auv_teleoperation

#endif 

