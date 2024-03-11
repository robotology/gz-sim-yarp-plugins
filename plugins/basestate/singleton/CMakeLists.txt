
add_library(gz-sim-yarp-basestate-singleton SHARED BaseStateDataSingleton.hh BaseStateDataSingleton.cc)

target_include_directories(gz-sim-yarp-basestate-singleton 
    PUBLIC  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

target_compile_features(gz-sim-yarp-basestate-singleton PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-basestate-singleton
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init)

install(TARGETS gz-sim-yarp-basestate-singleton)
