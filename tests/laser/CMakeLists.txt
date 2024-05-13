add_executable(LaserTest LaserTest.cc)
target_link_libraries(LaserTest
  GTest::gtest_main
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  ${YARP_LIBRARIES}
)
add_test(NAME LaserTest
         COMMAND LaserTest)

set(_env_vars)
list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-laser-system>")
list(APPEND _env_vars "LIBGL_ALWAYS_SOFTWARE=1")

set_tests_properties(LaserTest PROPERTIES
  ENVIRONMENT "${_env_vars}")

target_compile_definitions(LaserTest PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
