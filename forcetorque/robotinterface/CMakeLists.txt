
add_library(GazeboYarpRobotInterface SHARED GazeboYarpRobotInterface.cc)
set_property(TARGET GazeboYarpRobotInterface PROPERTY CXX_STANDARD 17)

target_link_libraries(GazeboYarpRobotInterface
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler)
