# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/UAV_demo/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/UAV_demo/build

# Include any dependencies generated for this target.
include crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/depend.make

# Include the progress variables for this target.
include crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/progress.make

# Include the compile flags for this target's objects.
include crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/flags.make

crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/src/comCheck.cpp.o: crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/flags.make
crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/src/comCheck.cpp.o: /home/pi/UAV_demo/src/crazyflie_ros/crazyflie_tools/src/comCheck.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/UAV_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/src/comCheck.cpp.o"
	cd /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_tools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/comCheck.dir/src/comCheck.cpp.o -c /home/pi/UAV_demo/src/crazyflie_ros/crazyflie_tools/src/comCheck.cpp

crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/src/comCheck.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/comCheck.dir/src/comCheck.cpp.i"
	cd /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/UAV_demo/src/crazyflie_ros/crazyflie_tools/src/comCheck.cpp > CMakeFiles/comCheck.dir/src/comCheck.cpp.i

crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/src/comCheck.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/comCheck.dir/src/comCheck.cpp.s"
	cd /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/UAV_demo/src/crazyflie_ros/crazyflie_tools/src/comCheck.cpp -o CMakeFiles/comCheck.dir/src/comCheck.cpp.s

# Object files for target comCheck
comCheck_OBJECTS = \
"CMakeFiles/comCheck.dir/src/comCheck.cpp.o"

# External object files for target comCheck
comCheck_EXTERNAL_OBJECTS =

/home/pi/UAV_demo/devel/lib/crazyflie_tools/comCheck: crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/src/comCheck.cpp.o
/home/pi/UAV_demo/devel/lib/crazyflie_tools/comCheck: crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/build.make
/home/pi/UAV_demo/devel/lib/crazyflie_tools/comCheck: /home/pi/UAV_demo/devel/lib/libcrazyflie_cpp.so
/home/pi/UAV_demo/devel/lib/crazyflie_tools/comCheck: /usr/lib/arm-linux-gnueabihf/libboost_program_options.so
/home/pi/UAV_demo/devel/lib/crazyflie_tools/comCheck: /usr/lib/arm-linux-gnueabihf/libusb-1.0.so
/home/pi/UAV_demo/devel/lib/crazyflie_tools/comCheck: crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/UAV_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pi/UAV_demo/devel/lib/crazyflie_tools/comCheck"
	cd /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/comCheck.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/build: /home/pi/UAV_demo/devel/lib/crazyflie_tools/comCheck

.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/build

crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/clean:
	cd /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_tools && $(CMAKE_COMMAND) -P CMakeFiles/comCheck.dir/cmake_clean.cmake
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/clean

crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/depend:
	cd /home/pi/UAV_demo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/UAV_demo/src /home/pi/UAV_demo/src/crazyflie_ros/crazyflie_tools /home/pi/UAV_demo/build /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_tools /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/comCheck.dir/depend

