add_executable(CameraTest CameraTest.cc)
target_link_libraries(CameraTest
  GTest::gtest_main
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  ${YARP_LIBRARIES}
)
add_test(NAME CameraTest COMMAND CameraTest)

set(_env_vars)
list(APPEND _env_vars
  "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-camera-system>"
  "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${CMAKE_CURRENT_SOURCE_DIR}"
  "LIBGL_ALWAYS_SOFTWARE=1"
)

set_tests_properties(CameraTest PROPERTIES
  ENVIRONMENT "${_env_vars}")

target_compile_definitions(CameraTest PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
