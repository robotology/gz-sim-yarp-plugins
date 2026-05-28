if(YARP_VERSION VERSION_GREATER_EQUAL "4.0.0")
  set(DEPTH_CAMERA_DEVICE_NAME "RGBDSensor_nws_yarp")
else()
  set(DEPTH_CAMERA_DEVICE_NAME "rgbdSensor_nws_yarp")
endif()

configure_file(stereodepthcamera_nws.xml.in stereodepthcamera_nws.xml @ONLY)

add_executable(StereoDepthCameraTest StereoDepthCameraTest.cc)
target_link_libraries(StereoDepthCameraTest
  GTest::gtest_main
  gz-sim::gz-sim
  gz-plugin::gz-plugin
  ${YARP_LIBRARIES}
  gz-sim-yarp-commons
)
add_test(NAME StereoDepthCameraTest COMMAND StereoDepthCameraTest)
set_tests_properties(StereoDepthCameraTest PROPERTIES
  ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz-sim-yarp-stereodepthcamera-system>"
  ENVIRONMENT_MODIFICATION "GZ_SIM_RESOURCE_PATH=path_list_append:${CMAKE_CURRENT_SOURCE_DIR}")

target_compile_definitions(StereoDepthCameraTest PRIVATE CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
