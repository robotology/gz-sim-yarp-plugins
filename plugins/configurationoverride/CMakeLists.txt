
add_library(gz-sim-yarp-configurationoverride-system SHARED ConfigurationOverride.cc)
target_compile_features(gz-sim-yarp-configurationoverride-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-configurationoverride-system
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::gz-sim
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-device-registry)

# Add install target
install(TARGETS gz-sim-yarp-configurationoverride-system)
