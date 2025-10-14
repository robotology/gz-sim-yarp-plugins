add_executable(LaserTest LaserTest.cc)
target_link_libraries(LaserTest
  GTest::gtest_main
  gz-sim::gz-sim
  gz-plugin::gz-plugin
  ${YARP_LIBRARIES}
)
add_test(NAME LaserTest
         COMMAND LaserTest)

set_tests_properties(LaserTest PROPERTIES
  ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-laser-system>"
  ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${CMAKE_CURRENT_SOURCE_DIR}")

target_compile_definitions(LaserTest PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
