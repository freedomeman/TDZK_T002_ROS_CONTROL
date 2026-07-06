add_library(evo_motors STATIC
  evo_motor_driver.cpp
)

target_include_directories(evo_motors
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/drivers/evo>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/protocol>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(evo_motors
  PUBLIC
    motors_protocol
    ${MOTORS_PUBLIC_DEPENDENCIES}
)
