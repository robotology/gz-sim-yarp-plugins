# This library is installed as it is a shared library, but no headers are installed,
# so the headers are effectively private headers, even if the library they belong to is installed
add_library(gz-sim-yarp-commons SHARED Common.hh ConfigurationHelpers.hh ConfigurationHelpers.cpp)

target_include_directories(gz-sim-yarp-commons PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

target_compile_features(gz-sim-yarp-commons PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-commons
  PUBLIC
    YARP::YARP_robotinterface
    YARP::YARP_os
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_init)

install(TARGETS gz-sim-yarp-commons
        EXPORT ${PROJECT_NAME})

install(FILES Common.hh ConfigurationHelpers.hh
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/gzyarp")
