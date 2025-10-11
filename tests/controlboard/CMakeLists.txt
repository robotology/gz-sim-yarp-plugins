set(TESTS
  ControlBoardTorqueControlTest
  ControlBoardPositionDirectControlTest
  ControlBoardPositionControlTest
  ControlBoardCommonsTest
  ControlBoardOnMultipleGazeboInstancesTest
  ControlBoardInitialConfigurationTest
)

foreach(TEST ${TESTS})
  add_executable(${TEST} ${TEST}.cc)
  if (GZ_SIM_YARP_PLUGINS_ENABLE_TESTS_WITH_ICUB_MAIN)
    target_compile_definitions(${TEST} PRIVATE GZ_SIM_YARP_PLUGINS_ENABLE_TESTS_WITH_ICUB_MAIN)
  endif()
  target_link_libraries(${TEST}
    PRIVATE GTest::gtest_main
            test-helpers
    PRIVATE gz-plugin::gz-plugin
    PRIVATE gz-sim::gz-sim
    PRIVATE
      YARP::YARP_dev
      YARP::YARP_os
      YARP::YARP_init
      gz-sim-yarp-commons
      gz-sim-yarp-device-registry
      gz-sim-yarp-controlboard-system)

  add_test(NAME ${TEST}
       COMMAND ${TEST})

  set_tests_properties(${TEST} PROPERTIES
    ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=path_list_append:$<TARGET_FILE_DIR:gz-sim-yarp-controlboard-system>"
    ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${CMAKE_CURRENT_SOURCE_DIR}")

  target_compile_definitions(${TEST} PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")


endforeach()
