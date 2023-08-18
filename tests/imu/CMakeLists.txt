add_executable(ImuTest ImuTest.cc)
target_link_libraries(ImuTest
  gtest_main
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  ${YARP_LIBRARIES}
)
add_test(NAME ImuTest
         COMMAND ImuTest)

set(_env_vars)
list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:GazeboYarpIMU>")

set_tests_properties(ImuTest PROPERTIES
  ENVIRONMENT "${_env_vars}")
