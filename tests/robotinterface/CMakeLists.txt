add_executable(RobotInterfaceTest RobotInterfaceTest.cc)
target_link_libraries(RobotInterfaceTest
  GTest::gtest_main
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  YARP::YARP_dev
)
add_test(NAME RobotInterfaceTest
         COMMAND RobotInterfaceTest)

set_tests_properties(RobotInterfaceTest PROPERTIES
  ENVIRONMENT_MODIFICATION "GZ_SIM_SYSTEM_PLUGIN_PATH=path_list_append:$<TARGET_FILE_DIR:gz-sim-yarp-robotinterface-system>"
  ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${CMAKE_CURRENT_SOURCE_DIR}")

target_compile_definitions(RobotInterfaceTest
  PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
