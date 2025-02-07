find_package(GTest REQUIRED)
# Only if the GZ_SIM_YARP_PLUGINS_ENABLE_TESTS_WITH_ICUB_MAIN variable is not defined,
# we look for ICUB, to set its default value
if(NOT DEFINED GZ_SIM_YARP_PLUGINS_ENABLE_TESTS_WITH_ICUB_MAIN)
  find_package(ICUB QUIET)
endif()
option(GZ_SIM_YARP_PLUGINS_ENABLE_TESTS_WITH_ICUB_MAIN "Enable tests that require the installation of the icub-main package" ICUB_FOUND)
if (GZ_SIM_YARP_PLUGINS_ENABLE_TESTS_WITH_ICUB_MAIN)
  find_package(ICUB REQUIRED)
endif()

add_subdirectory(forcetorque)
add_subdirectory(imu)
add_subdirectory(laser)
add_subdirectory(camera)
add_subdirectory(controlboard)
add_subdirectory(commons)
add_subdirectory(test-helpers)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
include(Fetchtiny-process-library)
add_subdirectory(clock)

if(GZ_SIM_YARP_PLUGINS_BUILD_TOOLS)
    add_subdirectory(gz-sim-yarp-plugins-check-model)
endif()
