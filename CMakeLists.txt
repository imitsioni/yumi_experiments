cmake_minimum_required(VERSION 2.8.3)
project(yumi_experiments)

find_package(
  catkin REQUIRED COMPONENTS
  rospy
  actionlib
  geometry_msgs
  actionlib_msgs
  controller_interface
  generic_control_toolbox
  folding_assembly_controller
  pluginlib
)

add_definitions(-std=c++11)
link_directories(${catkin_LIBRARY_DIRS})

add_action_files(
	DIRECTORY action
	FILES RunFolding.action ApproachController.action AdmittanceController.action SensorCalibration.action
)

generate_messages(
  DEPENDENCIES
	actionlib_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy actionlib actionlib_msgs folding_assembly_controller  controller_interface generic_control_toolbox geometry_msgs
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(approach_action_node src/approach_action_node.cpp src/approach_controller.cpp)
target_link_libraries(approach_action_node ${catkin_LIBRARIES})
add_dependencies(approach_action_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(admittance_action_node src/admittance_action_node.cpp src/admittance_controller.cpp)
target_link_libraries(admittance_action_node ${catkin_LIBRARIES})
add_dependencies(admittance_action_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(sensor_calibration_action_node src/sensor_calibration_action_node.cpp src/sensor_calibration_controller.cpp)
target_link_libraries(sensor_calibration_action_node ${catkin_LIBRARIES})
add_dependencies(sensor_calibration_action_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_library(admittance_ros src/admittance_ros.cpp)
target_link_libraries(admittance_ros ${catkin_LIBRARIES})

add_executable(testing_admittance src/testing_admittance.cpp)
target_link_libraries(testing_admittance ${catkin_LIBRARIES})

add_library(simple_controller src/simple_controller.cpp)
target_link_libraries(simple_controller ${catkin_LIBRARIES})
