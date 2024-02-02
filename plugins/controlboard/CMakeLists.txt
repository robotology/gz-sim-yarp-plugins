set(HEADER_FILES
    include/ControlBoard.hh
    include/ControlBoardDriver.hh
    include/ControlBoardDataSingleton.hh
)

set(SRC_FILES
    src/ControlBoard.cpp
    src/ControlBoardDriver.cpp
    src/ControlBoardDataSingleton.cpp
)

add_library(gz-sim-yarp-controlboard-system SHARED ${HEADER_FILES} ${SRC_FILES})

target_compile_features(gz-sim-yarp-controlboard-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-controlboard-system
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-handler)

# Add install target
install(TARGETS gz-sim-yarp-controlboard-system)
