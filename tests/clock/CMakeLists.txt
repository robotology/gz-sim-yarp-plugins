add_executable(ClockTest ClockTest.cc)
target_link_libraries(ClockTest
  GTest::gtest_main
  gz-sim${GZ_SIM_VER_OR_EMPTY}::gz-sim${GZ_SIM_VER_OR_EMPTY}
  gz-plugin${GZ_PLUGIN_VER_OR_EMPTY}::gz-plugin${GZ_PLUGIN_VER_OR_EMPTY}
  YARP::YARP_dev
  # As this test does not use localmode, launch yarpserver explicitly via tiny-process-library
  tiny-process-library
)
add_test(NAME ClockTest
         COMMAND ClockTest)

set(_env_vars)
list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-clock-system>")

set_tests_properties(ClockTest PROPERTIES
  ENVIRONMENT "${_env_vars}"
  ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${CMAKE_CURRENT_SOURCE_DIR}")


target_compile_definitions(ClockTest
  PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  PRIVATE YARP_SERVER_LOCATION="${YARP_INSTALL_PREFIX}/bin/yarpserver"
)
