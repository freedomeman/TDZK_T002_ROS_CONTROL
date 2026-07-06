add_library(encos_motors STATIC
  encos_motor_driver.cpp
)

target_include_directories(encos_motors
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/drivers/encos>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/protocol>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(encos_motors
  PUBLIC
    motors_protocol
    ${MOTORS_PUBLIC_DEPENDENCIES}
)
