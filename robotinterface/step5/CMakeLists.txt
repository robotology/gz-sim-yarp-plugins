if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
  cmake_minimum_required(VERSION 3.16)
endif()

project(forcetorque)
find_package(gz-cmake3 REQUIRED)
find_package(YARP REQUIRED COMPONENTS robotinterface os)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})


add_library(ForceTorque SHARED ForceTorque.cc)
add_library(ForceTorqueDriver SHARED ForceTorqueDriver.cpp)
add_library(Handler SHARED Handler.cc Handler.hh)
add_library(GazeboYarpRobotInterface SHARED GazeboYarpRobotInterface.cc)

set_property(TARGET ForceTorque PROPERTY CXX_STANDARD 17)
set_property(TARGET ForceTorqueDriver PROPERTY CXX_STANDARD 17)
set_property(TARGET Handler PROPERTY CXX_STANDARD 17)
set_property(TARGET GazeboYarpRobotInterface PROPERTY CXX_STANDARD 17)

target_link_libraries(ForceTorque
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    ForceTorqueDriver
    Handler)

target_link_libraries(ForceTorqueDriver
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler)

target_link_libraries(Handler
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init)



target_link_libraries(GazeboYarpRobotInterface
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler)
