add_executable(ForceTorqueTest ForceTorqueTest.cc)
target_link_libraries(ForceTorqueTest
  GTest::gtest_main
  gz-sim${GZ_SIM_VER_OR_EMPTY}::gz-sim${GZ_SIM_VER_OR_EMPTY}
  gz-plugin${GZ_PLUGIN_VER_OR_EMPTY}::gz-plugin${GZ_PLUGIN_VER_OR_EMPTY}
  ${YARP_LIBRARIES}
)
add_test(NAME ForceTorqueTest
         COMMAND ForceTorqueTest)
set_tests_properties(ForceTorqueTest PROPERTIES
  ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-forcetorque-system>"
  ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${CMAKE_CURRENT_SOURCE_DIR}")

target_compile_definitions(ForceTorqueTest PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
