cmake_minimum_required(VERSION 3.8)
project(openarm_gravcomp_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# TODO(HiroIshida): check if 
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  urdf
  kdl_parser
  orocos_kdl
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# TODO(HiroIshida): add parameter library generation??
# generate_parameter_library(
#   forward_command_controller_parameters
#   src/forward_command_controller_parameters.yaml
# )
# generate_parameter_library(
#   multi_interface_forward_command_controller_parameters
#   src/multi_interface_forward_command_controller_parameters.yaml
# )

add_library(openarm_gravcomp_controller SHARED src/gravcomp_controller.cpp)
target_include_directories(openarm_gravcomp_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/openarm_gravcomp_controller>
)
ament_target_dependencies(openarm_gravcomp_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(openarm_gravcomp_controller PRIVATE "OPENARM_GRAVCOMP_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface openarm_gravcomp_controller_plugin.xml)

ament_package()
