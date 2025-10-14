add_executable(DepthCameraTest DepthCameraTest.cc)
target_link_libraries(DepthCameraTest
  GTest::gtest_main
  gz-sim::gz-sim
  gz-plugin::gz-plugin
  ${YARP_LIBRARIES}
  gz-sim-yarp-commons
)
add_test(NAME DepthCameraTest COMMAND DepthCameraTest)
set_tests_properties(DepthCameraTest PROPERTIES
  ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-depthcamera-system>"
  ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${CMAKE_CURRENT_SOURCE_DIR}")

target_compile_definitions(DepthCameraTest PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
