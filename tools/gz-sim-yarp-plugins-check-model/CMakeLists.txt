add_executable(gz-sim-yarp-plugins-check-model gz-sim-yarp-plugins-check-model.cpp)
target_link_libraries(gz-sim-yarp-plugins-check-model gz-sim::gz-sim YARP::YARP_os gz-sim-yarp-plugins::gz-sim-yarp-device-registry CLI11::CLI11)

install(TARGETS gz-sim-yarp-plugins-check-model)
