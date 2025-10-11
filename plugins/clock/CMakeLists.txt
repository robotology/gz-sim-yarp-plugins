
add_library(gz-sim-yarp-clock-system SHARED Clock.cc)
target_compile_features(gz-sim-yarp-clock-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-clock-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER_OR_EMPTY}::gz-plugin${GZ_PLUGIN_VER_OR_EMPTY}
  PRIVATE gz-sim${GZ_SIM_VER_OR_EMPTY}::gz-sim${GZ_SIM_VER_OR_EMPTY}
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-device-registry)

# Add install target
install(TARGETS gz-sim-yarp-clock-system)
