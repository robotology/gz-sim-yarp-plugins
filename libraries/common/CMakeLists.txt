add_library(gz-sim-yarp-commons INTERFACE Common.hh)

target_include_directories(gz-sim-yarp-commons INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

install(TARGETS gz-sim-yarp-commons)
