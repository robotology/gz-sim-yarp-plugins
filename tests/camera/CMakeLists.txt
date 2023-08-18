add_executable(CameraTest CameraTest.cc)
target_link_libraries(CameraTest
  gtest_main
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  ${YARP_LIBRARIES}
)
add_test(NAME CameraTest COMMAND CameraTest)

set(_env_vars)
list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:GazeboYarpCamera>")
list(APPEND _env_vars "LIBGL_ALWAYS_SOFTWARE=1")

set_tests_properties(CameraTest PROPERTIES
  ENVIRONMENT "${_env_vars}")
