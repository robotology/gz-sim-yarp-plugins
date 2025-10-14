add_executable(ConfigurationParsingFromFileTest ConfigurationParsingFromFileTest.cc)
add_executable(ConfigurationParsingFromStringTest ConfigurationParsingFromStringTest.cc)
add_executable(ConcurrentInstancesTest ConcurrentInstancesTest.cc)

target_link_libraries(ConfigurationParsingFromFileTest
PRIVATE
  GTest::gtest_main
  test-helpers
  gz-sim::gz-sim
  gz-plugin::gz-plugin
  YARP::YARP_dev
  YARP::YARP_os
  YARP::YARP_init
  gz-sim-yarp-device-registry
  gz-sim-yarp-basestate-system
  gz-sim-yarp-camera-system
  gz-sim-yarp-controlboard-system
  gz-sim-yarp-forcetorque-system
  gz-sim-yarp-imu-system
  gz-sim-yarp-laser-system
)

target_link_libraries(ConfigurationParsingFromStringTest
PRIVATE
  GTest::gtest_main
  test-helpers
  gz-sim::gz-sim
  gz-plugin::gz-plugin
  YARP::YARP_dev
  YARP::YARP_os
  YARP::YARP_init
  gz-sim-yarp-device-registry
  gz-sim-yarp-basestate-system
  gz-sim-yarp-camera-system
  gz-sim-yarp-controlboard-system
  gz-sim-yarp-forcetorque-system
  gz-sim-yarp-imu-system
  gz-sim-yarp-laser-system
)

target_link_libraries(ConcurrentInstancesTest
PRIVATE
  GTest::gtest_main
  test-helpers
  gz-sim::gz-sim
  gz-plugin::gz-plugin
  YARP::YARP_dev
  YARP::YARP_os
  YARP::YARP_init
  gz-sim-yarp-device-registry
)

macro(gsyp_add_common_test testname)
    add_test(NAME ${testname} COMMAND ${testname})
    set_tests_properties(${testname} PROPERTIES
      ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-device-registry>"
      ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${CMAKE_CURRENT_SOURCE_DIR}/..")
    target_compile_definitions(${testname} PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endmacro()

gsyp_add_common_test(ConfigurationParsingFromFileTest)
gsyp_add_common_test(ConfigurationParsingFromStringTest)
gsyp_add_common_test(ConcurrentInstancesTest)
