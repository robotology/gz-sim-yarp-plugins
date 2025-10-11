add_executable(gz-sim-yarp-plugins-check-model gz-sim-yarp-plugins-check-model.cpp)
target_link_libraries(gz-sim-yarp-plugins-check-model gz-sim${GZ_SIM_VER_OR_EMPTY}::gz-sim${GZ_SIM_VER_OR_EMPTY} YARP::YARP_os gz-sim-yarp-plugins::gz-sim-yarp-device-registry CLI11::CLI11)

install(TARGETS gz-sim-yarp-plugins-check-model)
