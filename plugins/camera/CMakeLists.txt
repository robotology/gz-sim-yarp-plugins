project(camera)

add_subdirectory(singleton-camera)

add_library(GzYarpCamera SHARED Camera.cc CameraDriver.cpp)
target_compile_features(GzYarpCamera PRIVATE cxx_std_17)

target_link_libraries(GzYarpCamera
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler
    HandlerCamera)