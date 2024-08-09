add_executable(ForceTorqueTest ForceTorqueTest.cc)
target_link_libraries(ForceTorqueTest
  GTest::gtest_main
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  ${YARP_LIBRARIES}
)
add_test(NAME ForceTorqueTest
         COMMAND ForceTorqueTest)

set(_env_vars)
list(APPEND _env_vars
  "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-forcetorque-system>"
  "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:${CMAKE_CURRENT_SOURCE_DIR}"
)

set_tests_properties(ForceTorqueTest PROPERTIES
  ENVIRONMENT "${_env_vars}")

target_compile_definitions(ForceTorqueTest PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
