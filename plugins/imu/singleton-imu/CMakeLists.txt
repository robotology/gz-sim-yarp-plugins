
add_library(HandlerIMU SHARED Handler.cc Handler.hh)
target_compile_features(HandlerIMU PRIVATE cxx_std_17)

target_link_libraries(HandlerIMU
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init)

# Add install target
install(TARGETS HandlerIMU
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
