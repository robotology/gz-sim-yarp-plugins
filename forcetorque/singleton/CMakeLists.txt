
add_library(Handler SHARED Handler.cc Handler.hh)
set_property(TARGET Handler PROPERTY CXX_STANDARD 17)

target_link_libraries(Handler
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init)