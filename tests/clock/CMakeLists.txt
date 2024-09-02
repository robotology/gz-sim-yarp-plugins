add_executable(ClockTest ClockTest.cc)
target_link_libraries(ClockTest
  GTest::gtest_main
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  YARP::YARP_dev
  # As this test does not use localmode, launch yarpserver explicitly via tiny-process-library
  tiny-process-library
)
add_test(NAME ClockTest
         COMMAND ClockTest)

set(_env_vars)
list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-clock-system>")

set_tests_properties(ClockTest PROPERTIES
  ENVIRONMENT "${_env_vars}")

target_compile_definitions(ClockTest
  PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  PRIVATE YARP_SERVER_LOCATION="${YARP_INSTALL_PREFIX}/bin/yarpserver"
)
