add_library(gz-sim-yarp-handler SHARED Handler.cc Handler.hh)
target_include_directories(gz-sim-yarp-handler PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
target_compile_features(gz-sim-yarp-handler PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-handler
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init)

install(TARGETS gz-sim-yarp-handler)
