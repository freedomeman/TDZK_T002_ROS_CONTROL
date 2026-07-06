add_library(dm_motors STATIC
  dm_motor_driver.cpp
)

target_include_directories(dm_motors
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/drivers/dm>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/protocol>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(dm_motors
  PUBLIC
    motors_protocol
    ${MOTORS_PUBLIC_DEPENDENCIES}
)
