add_library(gz-sim-yarp-laser-system SHARED Laser.cc LaserDriver.cpp)

target_include_directories(gz-sim-yarp-laser-system
    PUBLIC  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

target_compile_features(gz-sim-yarp-laser-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-laser-system
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::gz-sim
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-device-registry
    gz-sim-yarp-commons)

# Add install target
install(TARGETS gz-sim-yarp-laser-system)
