cmake_minimum_required(VERSION 2.8.3)
project(auv_control)
find_package(catkin REQUIRED COMPONENTS auv_control_msgs dynamic_reconfigure rospy driver_base std_srvs auv_sensor_msgs)

catkin_python_setup()

# auto-generated dynamic reconfiguration GUI
generate_dynamic_reconfigure_options(cfg/AltitudeController.cfg
                                     cfg/DepthController.cfg
                                     cfg/TwistController.cfg)

catkin_package(
  CATKIN_DEPENDS auv_control_msgs auv_sensor_msgs)

