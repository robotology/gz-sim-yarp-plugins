
add_library(gz-sim-yarp-handler-basestate SHARED Handler.cc Handler.hh)
target_compile_features(gz-sim-yarp-handler-basestate PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-handler-basestate
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init)

install(TARGETS gz-sim-yarp-handler-basestate)
