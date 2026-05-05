add_executable(WorldinterfaceTest worldinterface.cc)
target_link_libraries(WorldinterfaceTest
  GTest::gtest_main
  gz-sim::gz-sim
  gz-plugin::gz-plugin
  ${YARP_LIBRARIES}
)
add_test(NAME WorldinterfaceTest
         COMMAND WorldinterfaceTest)

set_tests_properties(WorldinterfaceTest PROPERTIES
  ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-worldinterface-system>"
  ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${CMAKE_CURRENT_SOURCE_DIR}")

target_compile_definitions(WorldinterfaceTest PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
