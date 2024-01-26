
add_library(gz-sim-yarp-clock-system SHARED Clock.cc)
target_compile_features(gz-sim-yarp-clock-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-clock-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init)

# Add install target
install(TARGETS gz-sim-yarp-clock-system)
