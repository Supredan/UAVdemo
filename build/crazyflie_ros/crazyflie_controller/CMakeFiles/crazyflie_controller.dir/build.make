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
include crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/depend.make

# Include the progress variables for this target.
include crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/progress.make

# Include the compile flags for this target's objects.
include crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/flags.make

crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/src/controller.cpp.o: crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/flags.make
crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/src/controller.cpp.o: /home/pi/UAV_demo/src/crazyflie_ros/crazyflie_controller/src/controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/UAV_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/src/controller.cpp.o"
	cd /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_controller && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/crazyflie_controller.dir/src/controller.cpp.o -c /home/pi/UAV_demo/src/crazyflie_ros/crazyflie_controller/src/controller.cpp

crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/src/controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/crazyflie_controller.dir/src/controller.cpp.i"
	cd /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/UAV_demo/src/crazyflie_ros/crazyflie_controller/src/controller.cpp > CMakeFiles/crazyflie_controller.dir/src/controller.cpp.i

crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/src/controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/crazyflie_controller.dir/src/controller.cpp.s"
	cd /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/UAV_demo/src/crazyflie_ros/crazyflie_controller/src/controller.cpp -o CMakeFiles/crazyflie_controller.dir/src/controller.cpp.s

# Object files for target crazyflie_controller
crazyflie_controller_OBJECTS = \
"CMakeFiles/crazyflie_controller.dir/src/controller.cpp.o"

# External object files for target crazyflie_controller
crazyflie_controller_EXTERNAL_OBJECTS =

/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/src/controller.cpp.o
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/build.make
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/libtf.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/libtf2_ros.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/libactionlib.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/libmessage_filters.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/libroscpp.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/libtf2.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/librosconsole.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/librostime.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /opt/ros/melodic/lib/libcpp_common.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so.0.4
/home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller: crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/UAV_demo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller"
	cd /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_controller && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/crazyflie_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/build: /home/pi/UAV_demo/devel/lib/crazyflie_controller/crazyflie_controller

.PHONY : crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/build

crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/clean:
	cd /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_controller && $(CMAKE_COMMAND) -P CMakeFiles/crazyflie_controller.dir/cmake_clean.cmake
.PHONY : crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/clean

crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/depend:
	cd /home/pi/UAV_demo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/UAV_demo/src /home/pi/UAV_demo/src/crazyflie_ros/crazyflie_controller /home/pi/UAV_demo/build /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_controller /home/pi/UAV_demo/build/crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : crazyflie_ros/crazyflie_controller/CMakeFiles/crazyflie_controller.dir/depend
