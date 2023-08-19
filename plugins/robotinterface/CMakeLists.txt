
add_library(GzYarpRobotInterface SHARED GzYarpRobotInterface.cc)
target_compile_features(GzYarpRobotInterface PRIVATE cxx_std_17)

target_link_libraries(GzYarpRobotInterface
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler)
