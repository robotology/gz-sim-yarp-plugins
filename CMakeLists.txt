if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
  cmake_minimum_required(VERSION 3.16)
endif()

project(GazeboYARPPlugins)
find_package(gz-cmake3 REQUIRED)
find_package(YARP REQUIRED COMPONENTS robotinterface os)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

add_subdirectory(libraries)
add_subdirectory(plugins)
option(BUILD_TESTING "Create tests using CMake" ON)
if(BUILD_TESTING)
  include(CTest)
  add_subdirectory(tests)
endif()

