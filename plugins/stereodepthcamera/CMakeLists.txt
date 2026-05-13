add_library(gz-sim-yarp-stereodepthcamera-system SHARED
    StereoDepthCamera.cc
    ../depthcamera/DepthCameraDriver.cpp)

target_include_directories(gz-sim-yarp-stereodepthcamera-system
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../depthcamera>")

target_compile_features(gz-sim-yarp-stereodepthcamera-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-stereodepthcamera-system
    PRIVATE
        gz-plugin::gz-plugin
        gz-sim::gz-sim
        gz-sensors::gz-sensors
        YARP::YARP_os
        YARP::YARP_dev
        YARP::YARP_init
        gz-sim-yarp-device-registry
        gz-sim-yarp-commons)

install(TARGETS gz-sim-yarp-stereodepthcamera-system)
