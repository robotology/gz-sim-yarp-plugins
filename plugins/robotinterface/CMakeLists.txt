add_library(gz-sim-yarp-robotinterface-system SHARED RobotInterface.cc)

target_compile_features(gz-sim-yarp-robotinterface-system PRIVATE cxx_std_17)

target_link_libraries(gz-sim-yarp-robotinterface-system
  PRIVATE gz-plugin::gz-plugin
  PRIVATE gz-sim::gz-sim
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_robotinterface
    YARP::YARP_os
    YARP::YARP_init
    gz-sim-yarp-device-registry
    gz-sim-yarp-commons)

install(TARGETS gz-sim-yarp-robotinterface-system)
