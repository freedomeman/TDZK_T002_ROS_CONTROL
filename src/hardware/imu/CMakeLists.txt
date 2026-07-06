cmake_minimum_required(VERSION 3.8)
project(imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(pybind11 CONFIG QUIET)

set(IMU_PYTHON_SITE_PACKAGES
  "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
)

set(IMU_SOURCES
  src/imu_driver.cpp
  src/protocol/serial_port.cpp
  src/driver/yis320_dec.c
  src/driver/yis320_imu_driver.cpp
)

add_library(imu_core STATIC
  ${IMU_SOURCES}
)

target_include_directories(imu_core
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/driver>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/protocol>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(imu_core
  PUBLIC
    Threads::Threads
    spdlog::spdlog
)

if(TARGET pybind11::module)
  pybind11_add_module(imu_py src/pybind_module.cpp)
  target_link_libraries(imu_py PRIVATE imu_core)
  target_include_directories(imu_py PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
  )
else()
  message(STATUS "pybind11 not found, skipping imu_py module")
endif()

install(
  TARGETS imu_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TARGET imu_py)
  install(
    TARGETS imu_py
    LIBRARY DESTINATION ${IMU_PYTHON_SITE_PACKAGES}
    RUNTIME DESTINATION ${IMU_PYTHON_SITE_PACKAGES}
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES
    src/driver/yis320_dec.h
    src/driver/yis320_imu_driver.hpp
  DESTINATION include/driver
)

install(
  DIRECTORY src/protocol/
  DESTINATION include/protocol
  FILES_MATCHING
    PATTERN "*.hpp"
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
  Python3
  spdlog
)

ament_package()
