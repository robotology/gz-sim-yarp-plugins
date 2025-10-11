cmake_minimum_required(VERSION 3.22)

project(gz-sim-yarp-plugins
        LANGUAGES CXX C
        VERSION 0.5.1)

find_package(YCM REQUIRED)
find_package(YARP REQUIRED COMPONENTS robotinterface os)

if(YARP_VERSION VERSION_LESS "3.11.0")
    message(FATAL_ERROR "The minimum version of YARP required to build gz-sim-yarp-plugins is 3.11")
endif()

# Temporary set CMAKE_POLICY_VERSION_MINIMUM 3.5 as a workaround
# for https://github.com/robotology/gz-sim-yarp-plugins/pull/258#issue-2963247728
# Remove once we drop support for compilation on Ubuntu 24.04 with apt dependencies
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0.0")
    if(DEFINED CMAKE_POLICY_VERSION_MINIMUM)
        set(GSYP_CMAKE_POLICY_VERSION_MINIMUM_BACK ${CMAKE_POLICY_VERSION_MINIMUM})
    endif()
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

# Initial value of GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION is empty, and then is set to 8, 9 or >=10 depending if gz-sim8, gz-sim9 or gz-sim are available
set(GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION_DOCS "Version of gz-sim used to compile gz-sim-yarp-plugins")
set(GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION "" CACHE STRING ${GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION_DOCS})

# If GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION is not set, try to guess a suitable value
# based on the gz-sim version that can be found in the system
if(GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION STREQUAL "")
    find_package(gz-sim8 QUIET)
    if(gz-sim8_FOUND)
        set(GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION "8" CACHE STRING ${GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION_DOCS} FORCE)
    else()
        find_package(gz-sim9 QUIET)
        if(gz-sim9_FOUND)
            set(GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION "9" CACHE STRING ${GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION_DOCS} FORCE)
        else()
            find_package(gz-sim REQUIRED)
            set(GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION "${gz-sim_VERSION_MAJOR}" CACHE STRING ${GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION_DOCS} FORCE)
        endif()
    endif()
endif()

# This logic sets the GZ_SIM_VER_OR_EMPTY and GZ_PLUGIN_VER_OR_EMPTY variables.
# This is required as gz-sim8 and gz-sim9 (Harmonic and Ionic) still use versioned
# CMake packages and targets, while gz-sim >= 10 (Jetty and later) uses unversioned packages and targets.
# So GZ_SIM_VER_OR_EMPTY is 8 or 9 for gz-sim <= 9 and empty for gz-sim >= 10
# Similarly, GZ_PLUGIN_VER_OR_EMPTY is 2 for gz-sim8 (Harmonic), 3 for gz-sim9 (Ionic) and empty for gz-sim >= 10
if(GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION STREQUAL "8")
    find_package(gz-sim8 REQUIRED)
    find_package(gz-plugin2 REQUIRED COMPONENTS register)
    # gz-sim8 still uses versioned targets,
    # but we define an alias to versionless targets for keep the code simple
    add_library(gz-sim::gz-sim ALIAS gz-sim8::gz-sim8)
    add_library(gz-sensors::gz-sensors ALIAS gz-sensors8::gz-sensors8)
    add_library(gz-plugin::gz-plugin ALIAS gz-plugin2::gz-plugin2)
    set(GZ_SIM_CMAKE_PACKAGE "gz-sim8")
elseif(GZ_SIM_YARP_PLUGINS_USED_GZ_SIM_VERSION STREQUAL "9")
    find_package(gz-sim9 REQUIRED)
    find_package(gz-plugin3 REQUIRED COMPONENTS register)
    # gz-sim9 still uses versioned targets,
    # but we define an alias to versionless targets for keep the code simple
    add_library(gz-sim::gz-sim ALIAS gz-sim9::gz-sim9)
    add_library(gz-sensors::gz-sensors ALIAS gz-sensors9::gz-sensors9)
    add_library(gz-plugin::gz-plugin ALIAS gz-plugin3::gz-plugin3)
    set(GZ_SIM_CMAKE_PACKAGE "gz-sim9")
else()
    find_package(gz-sim REQUIRED)
    find_package(gz-plugin REQUIRED COMPONENTS register)
    set(GZ_SIM_CMAKE_PACKAGE "gz-sim")
endif()

option(GZ_SIM_YARP_PLUGINS_BUILD_TOOLS "If enabled, build command line helper tools" ON)

if(GZ_SIM_YARP_PLUGINS_BUILD_TOOLS)
    find_package(CLI11 REQUIRED)
endif()

if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0.0")
    if(DEFINED GSYP_CMAKE_POLICY_VERSION_MINIMUM_BACK)
        set(CMAKE_POLICY_VERSION_MINIMUM ${GSYP_CMAKE_POLICY_VERSION_MINIMUM_BACK})
    else()
        unset(CMAKE_POLICY_VERSION_MINIMUM)
    endif()
endif()

# Defines the CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR and many other useful macros.
# See https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# Generate the compile_commands.json file
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Tweak linker flags in Linux
if(UNIX AND NOT APPLE)
    if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
            if("${LINKER_BIN}" STREQUAL "ld")
                set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--unresolved-symbols=report-all")
            endif()
    endif()
endif()

# Enable RPATH support for installed binaries and libraries
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

# Support shared libraries on Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_subdirectory(libraries)
add_subdirectory(plugins)

if(GZ_SIM_YARP_PLUGINS_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

# Create and install CMake configuration files for this project that are
# necessary for other projects to call find_package(gz-sim-yarp-plugins).
include(InstallBasicPackageFiles)
Install_basic_package_files(${PROJECT_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY AnyNewerVersion
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            DEPENDENCIES "YARP COMPONENTS robotinterface os" "${GZ_SIM_CMAKE_PACKAGE}")

# Add the uninstall target
include(AddUninstallTarget)

option(BUILD_TESTING "Create tests using CMake" OFF)
if(BUILD_TESTING)
  include(CTest)
  add_subdirectory(tests)
endif()
