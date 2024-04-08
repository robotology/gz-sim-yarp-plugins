add_subdirectory(singleton)

add_library(gz-sim-yarp-forcetorque-system SHARED ForceTorque.cc ForceTorqueDriver.cpp)

target_include_directories(gz-sim-yarp-forcetorque-system
    PUBLIC  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

target_compile_features(gz-sim-yarp-forcetorque-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-forcetorque-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-device-registry
    gz-sim-yarp-forcetorque-singleton
    gz-sim-yarp-commons)

# Add install target
install(TARGETS gz-sim-yarp-forcetorque-system)
