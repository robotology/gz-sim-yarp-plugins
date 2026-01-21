add_library(gz-sim-yarp-worldinterface-system SHARED src/worldinterface.h src/worldinterface.cpp src/worldproxy.cpp)

target_include_directories(gz-sim-yarp-worldinterface-system PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

target_compile_features(gz-sim-yarp-worldinterface-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-worldinterface-system
  PRIVATE gz-plugin::gz-plugin
          gz-sim::gz-sim
          gz-sensors::gz-sensors
  
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-device-registry
    gz-sim-yarp-commons)

# Add install target
install(TARGETS gz-sim-yarp-worldinterface-system)
