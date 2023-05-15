if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
  cmake_minimum_required(VERSION 3.16)
endif()

find_package(gz-cmake3 REQUIRED)
find_package(YARP REQUIRED COMPONENTS os)
project(forcetorque)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

add_library(MyPlugin SHARED MyPlugin)
set_property(TARGET MyPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(MyPlugin
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

add_library(ForceTorque SHARED ForceTorque)
set_property(TARGET ForceTorque PROPERTY CXX_STANDARD 17)
target_link_libraries(ForceTorque
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    YARP::YARP_os
    YARP::YARP_init)