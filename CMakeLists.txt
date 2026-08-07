cmake_minimum_required(VERSION 3.0.2)
project(arm_ik_control)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 基础catkin组件，不含trac_ik
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  actionlib
  control_msgs
  kdl_parser
  urdf
  tf
  tf_conversions
  trac_ik_lib
  moveit_core
  moveit_ros_planning_interface
)


find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)


catkin_package(
  CATKIN_DEPENDS
    trac_ik_lib
  INCLUDE_DIRS include
  LIBRARIES arm_ik_control
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs trajectory_msgs control_msgs actionlib trac_ik kdl_parser urdf tf tf_conversions moveit_core moveit_ros_planning_interface
  DEPENDS
    Boost
    orocos_kdl
    pkg_nlopt
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${TRAC_IK_INCLUDE_DIR}
)


add_executable(ik_control_node src/ik_node.cpp)
target_link_libraries(ik_control_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${pkg_nlopt_LIBRARIES}
)
add_dependencies(ik_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 安装规则
install(TARGETS ik_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)