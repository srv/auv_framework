cmake_minimum_required(VERSION 2.8.3)
project(auv_teleoperation)
find_package(catkin REQUIRED COMPONENTS auv_control_msgs roscpp sensor_msgs std_msgs cmake_modules)

find_package(Eigen REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES joy_state
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS}) 

add_library(joy_state src/joy_state.cpp)
add_library(joy_dof_mapping src/joy_dof_mapping.cpp)

add_executable(teleoperation_joy_node 
                          src/teleoperation_joy_node.cpp
                          src/motor_policy.cpp
                          src/wrench_policy.cpp
                          src/altitude_control_wrench_policy.cpp
                          src/depth_control_wrench_policy.cpp
                          src/twist_policy.cpp
)
target_link_libraries(teleoperation_joy_node joy_state joy_dof_mapping ${catkin_LIBRARIES})


####################################
#   OJO AMB AIXÒ! Tret del manifest!
####################################
#<cpp lflags="-Wl,-rpath,${prefix}/lib -L${prefix}/lib -ljoy_state" cflags="-I${prefix}/include"/>
