add_executable(ImuTest ImuTest.cc)
target_link_libraries(ImuTest
  PRIVATE gtest_main
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE
        YARP::YARP_dev
        YARP::YARP_os
        YARP::YARP_init
        gz-sim-yarp-handler
)
add_test(NAME ImuTest
         COMMAND ImuTest)

set(_env_vars)
list(APPEND _env_vars "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-imu-system>")

set_tests_properties(ImuTest PROPERTIES
  ENVIRONMENT "${_env_vars}")
