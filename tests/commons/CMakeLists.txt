add_executable(ConfigurationParsingFromFileTest ConfigurationParsingFromFileTest.cc)
add_executable(ConfigurationParsingFromStringTest ConfigurationParsingFromStringTest.cc)
add_executable(ConcurrentInstancesTest ConcurrentInstancesTest.cc)

target_link_libraries(ConfigurationParsingFromFileTest
PRIVATE
  GTest::gtest_main
  test-helpers
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
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
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
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
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  YARP::YARP_dev
  YARP::YARP_os
  YARP::YARP_init
  gz-sim-yarp-device-registry
)


add_test(NAME ConfigurationParsingFromFileTest COMMAND ConfigurationParsingFromFileTest)
add_test(NAME ConfigurationParsingFromStringTest COMMAND ConfigurationParsingFromStringTest)
add_test(NAME ConcurrentInstancesTest COMMAND ConcurrentInstancesTest)

set(_env_vars)
list(APPEND _env_vars "LIBGL_ALWAYS_SOFTWARE=1"
  "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-device-registry>"
  "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${CMAKE_CURRENT_SOURCE_DIR}/.."
)

set_tests_properties(ConfigurationParsingFromFileTest PROPERTIES
  ENVIRONMENT "${_env_vars}")

set_tests_properties(ConfigurationParsingFromStringTest PROPERTIES
  ENVIRONMENT "${_env_vars}")

set_tests_properties(ConcurrentInstancesTest PROPERTIES
  ENVIRONMENT "${_env_vars}")
