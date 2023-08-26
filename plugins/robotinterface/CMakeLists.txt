
add_library(gz-sim-yarp-robotinterface-system SHARED RobotInterface.cc)
target_compile_features(gz-sim-yarp-robotinterface-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-robotinterface-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler)
