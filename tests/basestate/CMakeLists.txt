add_executable(BaseStateTest BaseStateTest.cc)
target_link_libraries(BaseStateTest
  GTest::gtest_main
  gz-sim${GZ_SIM_VER_OR_EMPTY}::gz-sim${GZ_SIM_VER_OR_EMPTY}
  gz-plugin${GZ_PLUGIN_VER_OR_EMPTY}::gz-plugin${GZ_PLUGIN_VER_OR_EMPTY}
  YARP::YARP_dev
  YARP::YARP_os
  YARP::YARP_init
  YARP::YARP_sig
  gz-sim-yarp-device-registry
  gz-sim-yarp-basestate-system
  gz-sim-yarp-commons
  test-helpers
)
add_test(NAME BaseStateTest
         COMMAND BaseStateTest
         ENVIRONMENT "YARP_PORT_PREFIX=BaseStateTest_${CMAKE_RANDOM_UUID}")

set_tests_properties(BaseStateTest PROPERTIES
  ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-basestate-system>"
  ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${CMAKE_CURRENT_SOURCE_DIR}"
)

target_compile_definitions(BaseStateTest PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
