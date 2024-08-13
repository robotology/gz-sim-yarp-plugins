find_package(GTest REQUIRED)

add_subdirectory(forcetorque)
add_subdirectory(imu)
add_subdirectory(laser)
add_subdirectory(camera)
add_subdirectory(clock)
add_subdirectory(controlboard)
add_subdirectory(commons)
add_subdirectory(test-helpers)

if(GZ_SIM_YARP_PLUGINS_BUILD_TOOLS)
    add_subdirectory(gz-sim-yarp-plugins-check-model)
endif()
