cmake_minimum_required(VERSION 3.8)
project(robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(imu REQUIRED)
find_package(motors REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

add_library(robot_hardware_node SHARED
  src/robot_hardware_node.cpp
)

target_include_directories(robot_hardware_node
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_hardware_node
  hardware_interface
  imu
  motors
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(robot_hardware_node motors::motors_core imu::imu_core Threads::Threads)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware_plugin.xml)

install(
  TARGETS robot_hardware_node
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
  FILES robot_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  ament_cmake
  hardware_interface
  imu
  motors
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
