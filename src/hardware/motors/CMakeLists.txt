cmake_minimum_required(VERSION 3.8)
project(motors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(MOTORS_BUILD_PYTHON "Build the pybind11 module for motors" ON)

find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(pybind11 CONFIG QUIET)

set(MOTORS_PYTHON_SITE_PACKAGES
  "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
)

set(MOTORS_PUBLIC_DEPENDENCIES
  Threads::Threads
  spdlog::spdlog
)

add_subdirectory(src/protocol)
add_subdirectory(src/drivers/dm)
add_subdirectory(src/drivers/evo)
add_subdirectory(src/drivers/encos)
add_subdirectory(src/drivers/stw)

add_library(motors_core STATIC
  src/motor_driver.cpp
)

target_include_directories(motors_core
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/drivers/dm>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/drivers/evo>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/drivers/encos>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/drivers/stw>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/protocol>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(motors_core
  PUBLIC
    dm_motors
    evo_motors
    encos_motors
    stw_motors
    ${MOTORS_PUBLIC_DEPENDENCIES}
)

if(MOTORS_BUILD_PYTHON AND TARGET pybind11::module)
  pybind11_add_module(motors_py src/pybind_module.cpp)
  target_link_libraries(motors_py PRIVATE motors_core)
  target_include_directories(motors_py PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
  )
else()
  message(STATUS "pybind11 not found, skipping motors_py module")
endif()

install(
  TARGETS motors_protocol dm_motors evo_motors encos_motors stw_motors motors_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(TARGET motors_py)
  install(
    TARGETS motors_py
    LIBRARY DESTINATION ${MOTORS_PYTHON_SITE_PACKAGES}
    RUNTIME DESTINATION ${MOTORS_PYTHON_SITE_PACKAGES}
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY src/protocol/
  DESTINATION include/protocol
  FILES_MATCHING PATTERN "*.hpp"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  ament_cmake
  spdlog
)

ament_package()
