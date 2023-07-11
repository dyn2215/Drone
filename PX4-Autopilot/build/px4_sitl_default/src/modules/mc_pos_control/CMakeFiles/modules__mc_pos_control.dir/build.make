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

# Include any dependencies generated for this target.
include src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/depend.make

# Include the progress variables for this target.
include src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/flags.make

src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.o: src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/flags.make
src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.o: ../../src/modules/mc_pos_control/MulticopterPositionControl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.cpp

src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.cpp > CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.i

src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control/MulticopterPositionControl.cpp -o CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.s

# Object files for target modules__mc_pos_control
modules__mc_pos_control_OBJECTS = \
"CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.o"

# External object files for target modules__mc_pos_control
modules__mc_pos_control_EXTERNAL_OBJECTS =

src/modules/mc_pos_control/libmodules__mc_pos_control.a: src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/MulticopterPositionControl.cpp.o
src/modules/mc_pos_control/libmodules__mc_pos_control.a: src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/build.make
src/modules/mc_pos_control/libmodules__mc_pos_control.a: src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmodules__mc_pos_control.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control && $(CMAKE_COMMAND) -P CMakeFiles/modules__mc_pos_control.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__mc_pos_control.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/build: src/modules/mc_pos_control/libmodules__mc_pos_control.a

.PHONY : src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/build

src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control && $(CMAKE_COMMAND) -P CMakeFiles/modules__mc_pos_control.dir/cmake_clean.cmake
.PHONY : src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/clean

src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/mc_pos_control/CMakeFiles/modules__mc_pos_control.dir/depend

