
add_library(gz-sim-yarp-controlboard-singleton SHARED ControlBoardData.hh ControlBoardDataSingleton.hh ControlBoardTrajectory.hh ControlBoardDataSingleton.cpp ControlBoardTrajectory.cpp)

target_include_directories(gz-sim-yarp-controlboard-singleton PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

target_compile_features(gz-sim-yarp-controlboard-singleton PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-controlboard-singleton
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init)

install(TARGETS gz-sim-yarp-controlboard-singleton)
