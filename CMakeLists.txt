cmake_minimum_required(VERSION 3.22)

project(gz-sim-yarp-plugins
        LANGUAGES CXX C
        VERSION 0.3.0)

find_package(gz-cmake3 REQUIRED)
find_package(YARP REQUIRED COMPONENTS robotinterface os)
find_package(YCM REQUIRED)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

option(GZ_SIM_YARP_PLUGINS_BUILD_TOOLS "If enabled, build command line helper tools" ON)

if(GZ_SIM_YARP_PLUGINS_BUILD_TOOLS)
    find_package(CLI11 REQUIRED)
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
                            DEPENDENCIES "YARP" "gz-sim${GZ_SIM_VER}")

# Add the uninstall target
include(AddUninstallTarget)

option(BUILD_TESTING "Create tests using CMake" OFF)
if(BUILD_TESTING)
  include(CTest)
  add_subdirectory(tests)
endif()
