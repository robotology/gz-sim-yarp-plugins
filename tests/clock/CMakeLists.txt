add_executable(ClockTest ClockTest.cc)
target_link_libraries(ClockTest
  GTest::gtest_main
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  YARP::YARP_dev
)
add_test(NAME ClockTest
         COMMAND ClockTest)

set(_env_vars)
list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-clock-system>")

set_tests_properties(ClockTest PROPERTIES
  ENVIRONMENT "${_env_vars}")
