project(laser)

add_subdirectory(singleton-laser)

add_library(GazeboYarpLaser SHARED Laser.cc LaserDriver.cpp)
target_compile_features(GazeboYarpLaser PRIVATE cxx_std_17)

target_link_libraries(GazeboYarpLaser
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler
    HandlerLaser)
