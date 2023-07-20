project(forcetorque)

add_subdirectory(singleton-forcetorque)

add_library(GazeboYarpForceTorque SHARED ForceTorque.cc ForceTorqueDriver.cpp)
target_compile_features(GazeboYarpForceTorque PRIVATE cxx_std_17)

target_link_libraries(GazeboYarpForceTorque
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler
    HandlerForceTorque)
