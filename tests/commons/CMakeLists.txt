add_executable(ConfigurationParsingTest ConfigurationParsingTest.cc)

target_link_libraries(ConfigurationParsingTest
  gtest_main
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  YARP::YARP_dev
  YARP::YARP_os
  YARP::YARP_init
  gz-sim-yarp-commons
  gz-sim-yarp-handler
  gz-sim-yarp-basestate-system
  gz-sim-yarp-camera-system
  gz-sim-yarp-controlboard-system
  gz-sim-yarp-forcetorque-system
  gz-sim-yarp-imu-system
  gz-sim-yarp-laser-system
  gz-sim-yarp-robotinterface-system
  )

add_test(NAME ConfigurationParsingTest COMMAND ConfigurationParsingTest)

set(_env_vars)
# list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-camera-system>")
list(APPEND _env_vars "LIBGL_ALWAYS_SOFTWARE=1" "GZ_SIM_RESOURCE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/..")

set_tests_properties(ConfigurationParsingTest PROPERTIES
  ENVIRONMENT "${_env_vars}")
