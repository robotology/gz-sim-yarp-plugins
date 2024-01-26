
add_subdirectory(singleton-basestate)

add_library(gz-sim-yarp-basestate-system SHARED BaseState.cc BaseStateDriver.cpp)
target_compile_features(gz-sim-yarp-basestate-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-basestate-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-handler
    gz-sim-yarp-handler-basestate)

# Add install target
install(TARGETS gz-sim-yarp-basestate-system)
