add_library(gz-sim-yarp-camera-system SHARED Camera.cc CameraDriver.cpp)

target_include_directories(gz-sim-yarp-camera-system PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

target_compile_features(gz-sim-yarp-camera-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-camera-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER_OR_EMPTY}::gz-plugin${GZ_PLUGIN_VER_OR_EMPTY}
  PRIVATE gz-sim${GZ_SIM_VER_OR_EMPTY}::gz-sim${GZ_SIM_VER_OR_EMPTY}
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-device-registry
    gz-sim-yarp-commons)

# Add install target
install(TARGETS gz-sim-yarp-camera-system)
