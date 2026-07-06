cmake_minimum_required(VERSION 3.8)
project(t002_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIR
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(t002_controller SHARED
  src/t002_controller.cpp
  src/user_code.cpp
)

target_include_directories(t002_controller
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DIR}
)

ament_target_dependencies(t002_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  geometry_msgs
  sensor_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(controller_interface t002_controller_plugin.xml)

# xbox_text 独立节点
add_executable(xbox_text src/xbox_text.cpp)
target_include_directories(xbox_text PRIVATE ${THIS_PACKAGE_INCLUDE_DIR})
ament_target_dependencies(xbox_text
  rclcpp
  std_msgs
)

install(TARGETS xbox_text
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS t002_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

ament_package()
