set(TESTS
  ControlBoardTorqueControlTest
  ControlBoardPositionDirectControlTest
  ControlBoardPositionControlTest
  ControlBoardCommonsTest
  ControlBoardOnMultipleGazeboInstancesTest
)

foreach(TEST ${TESTS})
  add_executable(${TEST} ${TEST}.cc)

  target_link_libraries(${TEST}
    PRIVATE GTest::gtest_main
            test-helpers
    PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
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
    ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-controlboard-system>")
    
endforeach()
