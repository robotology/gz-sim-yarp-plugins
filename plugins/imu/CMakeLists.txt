project(imu)

add_subdirectory(singleton-imu)

add_library(gz-sim-yarp-imu-system SHARED IMU.cc IMUDriver.cpp)
target_compile_features(gz-sim-yarp-imu-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-imu-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler
    HandlerIMU)
