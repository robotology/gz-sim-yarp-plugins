add_executable(ForceTorqueTest ForceTorqueTest.cc)
target_link_libraries(ForceTorqueTest
  gtest_main
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  ${YARP_LIBRARIES}
)
add_test(NAME ForceTorqueTest
         COMMAND ForceTorqueTest)

set(_env_vars)
list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:GazeboYarpForceTorque>")

set_tests_properties(ForceTorqueTest PROPERTIES
  ENVIRONMENT "${_env_vars}")
