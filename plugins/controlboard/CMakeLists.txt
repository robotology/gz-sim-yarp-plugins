set(HEADER_FILES
    include/ControlBoard.hh
    include/ControlBoardData.hh
    include/ControlBoardDriver.hh
    include/ControlBoardTrajectory.hh
)

set(SRC_FILES
    src/ControlBoard.cpp
    src/ControlBoardDriver.cpp
    src/ControlBoardTrajectory.cpp
)

add_library(gz-sim-yarp-controlboard-system SHARED ${HEADER_FILES} ${SRC_FILES})

target_include_directories(gz-sim-yarp-controlboard-system 
    PUBLIC  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

target_compile_features(gz-sim-yarp-controlboard-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-controlboard-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-commons
    gz-sim-yarp-device-registry
)

# Add install target
install(TARGETS gz-sim-yarp-controlboard-system)
