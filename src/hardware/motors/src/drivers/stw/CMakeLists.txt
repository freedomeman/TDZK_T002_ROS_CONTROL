add_library(stw_motors STATIC
  stw_motor_driver.cpp
)

target_include_directories(stw_motors
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/drivers/stw>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/protocol>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(stw_motors
  PUBLIC
    motors_protocol
    ${MOTORS_PUBLIC_DEPENDENCIES}
)
