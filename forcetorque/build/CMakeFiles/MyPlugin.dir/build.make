# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque/build"

# Include any dependencies generated for this target.
include CMakeFiles/MyPlugin.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/MyPlugin.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/MyPlugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MyPlugin.dir/flags.make

CMakeFiles/MyPlugin.dir/MyPlugin.cc.o: CMakeFiles/MyPlugin.dir/flags.make
CMakeFiles/MyPlugin.dir/MyPlugin.cc.o: ../MyPlugin.cc
CMakeFiles/MyPlugin.dir/MyPlugin.cc.o: CMakeFiles/MyPlugin.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MyPlugin.dir/MyPlugin.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/MyPlugin.dir/MyPlugin.cc.o -MF CMakeFiles/MyPlugin.dir/MyPlugin.cc.o.d -o CMakeFiles/MyPlugin.dir/MyPlugin.cc.o -c "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque/MyPlugin.cc"

CMakeFiles/MyPlugin.dir/MyPlugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MyPlugin.dir/MyPlugin.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque/MyPlugin.cc" > CMakeFiles/MyPlugin.dir/MyPlugin.cc.i

CMakeFiles/MyPlugin.dir/MyPlugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MyPlugin.dir/MyPlugin.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque/MyPlugin.cc" -o CMakeFiles/MyPlugin.dir/MyPlugin.cc.s

# Object files for target MyPlugin
MyPlugin_OBJECTS = \
"CMakeFiles/MyPlugin.dir/MyPlugin.cc.o"

# External object files for target MyPlugin
MyPlugin_EXTERNAL_OBJECTS =

libMyPlugin.so: CMakeFiles/MyPlugin.dir/MyPlugin.cc.o
libMyPlugin.so: CMakeFiles/MyPlugin.dir/build.make
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-sim7.so.7.4.0
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-profiler.so.5.3.1
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-fuel_tools8.so.8.0.2
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-gui7.so.7.1.0
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-events.so.5.3.1
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5.so.5.3.1
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-plugin2-loader.so.2.0.1
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-plugin2.so.2.0.1
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5QuickControls2.so.5.15.3
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Quick.so.5.15.3
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5QmlModels.so.5.15.3
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Qml.so.5.15.3
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Network.so.5.15.3
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.3
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.3
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.3
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-transport12-parameters.so.12.1.0
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-transport12.so.12.1.0
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-msgs9.so.9.4.0
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libsdformat13.so.13.4.1
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-math7.so.7.1.0
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-utils2.so.2.0.0
libMyPlugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libMyPlugin.so: CMakeFiles/MyPlugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libMyPlugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MyPlugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MyPlugin.dir/build: libMyPlugin.so
.PHONY : CMakeFiles/MyPlugin.dir/build

CMakeFiles/MyPlugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MyPlugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MyPlugin.dir/clean

CMakeFiles/MyPlugin.dir/depend:
	cd "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque" "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque" "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque/build" "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque/build" "/mnt/c/Users/lucap/OneDrive - unige.it/Documenti/Università/Magistrale/Software Engineering/SE_Project/study-gazebo-garden-yarp-plugins/forcetorque/build/CMakeFiles/MyPlugin.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/MyPlugin.dir/depend

