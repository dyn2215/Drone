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

# Utility rule file for none_cloudship_callgrind.

# Include the progress variables for this target.
include platforms/posix/CMakeFiles/none_cloudship_callgrind.dir/progress.make

platforms/posix/CMakeFiles/none_cloudship_callgrind:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/tmp && /home/nics/Drone/PX4-Autopilot/Tools/sitl_run.sh /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin/px4 callgrind none cloudship none /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default

none_cloudship_callgrind: platforms/posix/CMakeFiles/none_cloudship_callgrind
none_cloudship_callgrind: platforms/posix/CMakeFiles/none_cloudship_callgrind.dir/build.make

.PHONY : none_cloudship_callgrind

# Rule to build all files generated by this target.
platforms/posix/CMakeFiles/none_cloudship_callgrind.dir/build: none_cloudship_callgrind

.PHONY : platforms/posix/CMakeFiles/none_cloudship_callgrind.dir/build

platforms/posix/CMakeFiles/none_cloudship_callgrind.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix && $(CMAKE_COMMAND) -P CMakeFiles/none_cloudship_callgrind.dir/cmake_clean.cmake
.PHONY : platforms/posix/CMakeFiles/none_cloudship_callgrind.dir/clean

platforms/posix/CMakeFiles/none_cloudship_callgrind.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/platforms/posix /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/CMakeFiles/none_cloudship_callgrind.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/posix/CMakeFiles/none_cloudship_callgrind.dir/depend

