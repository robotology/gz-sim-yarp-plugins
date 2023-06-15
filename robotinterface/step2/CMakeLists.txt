if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
  cmake_minimum_required(VERSION 3.16)
endif()

project(step2)
find_package(YARP REQUIRED COMPONENTS robotinterface)


add_executable(launch launch.cc)
target_link_libraries(launch
  PRIVATE
    ${YARP_LIBRARIES}
    YARP::YARP_os
    YARP::YARP_init)
