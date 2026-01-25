cmake_minimum_required(VERSION 3.16)
project(delta_kinematics_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  geometry_msgs
  hardware_interface
  kinematics_interface
  kinematics_interface_delta
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(ament_cmake_gtest QUIET)

# Try to locate kinematics_interface headers (some installs use nested include paths)
find_path(KINEMATICS_INTERFACE_INCLUDE_DIR
  NAMES kinematics_interface/kinematics_interface/kinematics_interface.hpp
  PATHS ${CMAKE_PREFIX_PATH}/include /opt/ros/jazzy/include /usr/include
)

# Controller library
add_library(delta_kinematics_controller SHARED
  src/delta_kinematics_controller.cpp
)
target_compile_features(delta_kinematics_controller PUBLIC cxx_std_17)

target_include_directories(delta_kinematics_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/delta_kinematics_controller>
)

if(KINEMATICS_INTERFACE_INCLUDE_DIR)
  target_include_directories(delta_kinematics_controller PUBLIC ${KINEMATICS_INTERFACE_INCLUDE_DIR})
endif()

target_link_libraries(delta_kinematics_controller PUBLIC
  Eigen3::Eigen
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  kinematics_interface::kinematics_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

ament_target_dependencies(delta_kinematics_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Standalone demo node (optional)
add_executable(delta_controller_node
  src/delta_controller_node.cpp
)
target_compile_features(delta_controller_node PUBLIC cxx_std_17)
target_include_directories(delta_controller_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
if(KINEMATICS_INTERFACE_INCLUDE_DIR)
  target_include_directories(delta_controller_node PUBLIC ${KINEMATICS_INTERFACE_INCLUDE_DIR})
endif()
target_link_libraries(delta_controller_node PUBLIC
  kinematics_interface::kinematics_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

# Export plugin description for controller_manager discovery
pluginlib_export_plugin_description_file(controller_interface resource/delta_kinematics_controller_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
endif()

install(
  DIRECTORY include/
  DESTINATION include/delta_kinematics_controller
)

install(TARGETS
  delta_kinematics_controller
  delta_controller_node
  EXPORT export_delta_kinematics_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(FILES
  resource/delta_kinematics_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  urdf
  description
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_delta_kinematics_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Eigen3)
ament_package()
