set(TESTS
  ControlBoardTorqueControlTest
  ControlBoardPositionDirectControlTest
  ControlBoardPositionControlTest
  ControlBoardCommonsTest
)

foreach(TEST ${TESTS})
  add_executable(${TEST} ${TEST}.cc)

  target_link_libraries(${TEST}
    PRIVATE gtest_main
    PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    PRIVATE
      YARP::YARP_dev
      YARP::YARP_os
      YARP::YARP_init
      gz-sim-yarp-handler
  )

  add_test(NAME ${TEST}
       COMMAND ${TEST})

  set_tests_properties(${TEST} PROPERTIES
    ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-controlboard-system>")
endforeach()
