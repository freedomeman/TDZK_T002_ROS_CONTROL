add_library(motors_protocol STATIC
  can/socket_can.cpp
)

target_include_directories(motors_protocol
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/protocol>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/protocol/can>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(motors_protocol
  PUBLIC
    ${MOTORS_PUBLIC_DEPENDENCIES}
)
