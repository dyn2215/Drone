# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/nics/Drone/PX4-Autopilot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default

# Utility rule file for gazebo_tiltrotor_valgrind_mcmillan_airfield.

# Include the progress variables for this target.
include platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield.dir/progress.make

platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/tmp && /home/nics/Drone/PX4-Autopilot/Tools/sitl_run.sh /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin/px4 valgrind gazebo tiltrotor mcmillan_airfield /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default

gazebo_tiltrotor_valgrind_mcmillan_airfield: platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield
gazebo_tiltrotor_valgrind_mcmillan_airfield: platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield.dir/build.make

.PHONY : gazebo_tiltrotor_valgrind_mcmillan_airfield

# Rule to build all files generated by this target.
platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield.dir/build: gazebo_tiltrotor_valgrind_mcmillan_airfield

.PHONY : platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield.dir/build

platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix && $(CMAKE_COMMAND) -P CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield.dir/cmake_clean.cmake
.PHONY : platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield.dir/clean

platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/platforms/posix /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/posix/CMakeFiles/gazebo_tiltrotor_valgrind_mcmillan_airfield.dir/depend

