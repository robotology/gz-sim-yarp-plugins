project(laser)

add_subdirectory(singleton-laser)

add_library(gz-sim-yarp-laser-system SHARED Laser.cc LaserDriver.cpp)
target_compile_features(gz-sim-yarp-laser-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-laser-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init
    Handler
    HandlerLaser)

# Add install target
install(TARGETS gz-sim-yarp-laser-system
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
