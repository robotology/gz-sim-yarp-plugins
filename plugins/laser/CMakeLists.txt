project(laser)

add_subdirectory(singleton-laser)

add_library(GzYarpLaser SHARED Laser.cc LaserDriver.cpp)
target_compile_features(GzYarpLaser PRIVATE cxx_std_17)

target_link_libraries(GzYarpLaser
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler
    HandlerLaser)
