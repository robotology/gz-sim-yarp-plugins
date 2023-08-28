
add_library(gz-sim-yarp-handler-forcetorque SHARED Handler.cc Handler.hh)
target_compile_features(HandlerForceTorque PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-handler-forcetorque
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init)

install(TARGETS gz-sim-yarp-handler-forcetorque
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
