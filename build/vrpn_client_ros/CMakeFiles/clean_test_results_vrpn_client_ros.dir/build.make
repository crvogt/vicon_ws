# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/carson/vicon_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/carson/vicon_ws/build

# Utility rule file for clean_test_results_vrpn_client_ros.

# Include the progress variables for this target.
include vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros.dir/progress.make

vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros:
	cd /home/carson/vicon_ws/build/vrpn_client_ros && /usr/bin/python /opt/ros/indigo/share/catkin/cmake/test/remove_test_results.py /home/carson/vicon_ws/build/test_results/vrpn_client_ros

clean_test_results_vrpn_client_ros: vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros
clean_test_results_vrpn_client_ros: vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros.dir/build.make
.PHONY : clean_test_results_vrpn_client_ros

# Rule to build all files generated by this target.
vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros.dir/build: clean_test_results_vrpn_client_ros
.PHONY : vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros.dir/build

vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros.dir/clean:
	cd /home/carson/vicon_ws/build/vrpn_client_ros && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_vrpn_client_ros.dir/cmake_clean.cmake
.PHONY : vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros.dir/clean

vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros.dir/depend:
	cd /home/carson/vicon_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/carson/vicon_ws/src /home/carson/vicon_ws/src/vrpn_client_ros /home/carson/vicon_ws/build /home/carson/vicon_ws/build/vrpn_client_ros /home/carson/vicon_ws/build/vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vrpn_client_ros/CMakeFiles/clean_test_results_vrpn_client_ros.dir/depend

