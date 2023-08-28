
add_library(gz-sim-yarp-handler-imu SHARED Handler.cc Handler.hh)
target_compile_features(HandlerIMU PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-handler-imu
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init)

# Add install target
install(TARGETS gz-sim-yarp-handler-imu
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
