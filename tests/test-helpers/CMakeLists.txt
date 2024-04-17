add_library(test-helpers INTERFACE TestHelpers.hh)

target_include_directories(test-helpers INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

target_link_libraries(test-helpers
  INTERFACE GTest::gtest_main
            gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
            gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
            YARP::YARP_dev
            YARP::YARP_os
            YARP::YARP_init
            gz-sim-yarp-device-registry
)
