cmake_minimum_required(VERSION 2.8.3)
project(planar_robot_pkgs)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(FILES Planar_CTCMsg.msg)
add_message_files(FILES Pub_torque_calc_Msg.msg)
add_message_files(FILES Pub_torque_calc_Msg2.msg)

generate_messages(DEPENDENCIES std_msgs)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS} ${EIGEN3_INCLUDE_DIR} ${RBDL_INCLUDE_DIR})

add_library(${PROJECT_NAME} src/plugin.cc src/Constraints.cc src/Dynamics.cc src/Joint.cc src/Kinematics.cc src/Logging.cc src/Model.cc src/rbdl_mathutils.cc src/rbdl_utils.cc src/rbdl_version.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${RBDL_LIBRARY})

add_executable(torque_calc src/torque_calc.cc src/Constraints.cc src/Dynamics.cc src/Joint.cc src/Kinematics.cc src/Logging.cc src/Model.cc src/rbdl_mathutils.cc src/rbdl_utils.cc src/rbdl_version.cc)
add_dependencies(torque_calc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(torque_calc ${catkin_LIBRARIES} ${RBDL_LIBRARY})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} rbdl
 DEPENDS gazebo_ros roscpp rospy std_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

