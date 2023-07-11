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
include src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/depend.make

# Include the progress variables for this target.
include src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/flags.make

src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/PositionControl.cpp.o: src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/flags.make
src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/PositionControl.cpp.o: ../../src/modules/mc_pos_control/PositionControl/PositionControl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/PositionControl.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PositionControl.dir/PositionControl.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/PositionControl.cpp

src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/PositionControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PositionControl.dir/PositionControl.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/PositionControl.cpp > CMakeFiles/PositionControl.dir/PositionControl.cpp.i

src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/PositionControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PositionControl.dir/PositionControl.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/PositionControl.cpp -o CMakeFiles/PositionControl.dir/PositionControl.cpp.s

src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/ControlMath.cpp.o: src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/flags.make
src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/ControlMath.cpp.o: ../../src/modules/mc_pos_control/PositionControl/ControlMath.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/ControlMath.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/PositionControl.dir/ControlMath.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/ControlMath.cpp

src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/ControlMath.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/PositionControl.dir/ControlMath.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/ControlMath.cpp > CMakeFiles/PositionControl.dir/ControlMath.cpp.i

src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/ControlMath.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/PositionControl.dir/ControlMath.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/ControlMath.cpp -o CMakeFiles/PositionControl.dir/ControlMath.cpp.s

# Object files for target PositionControl
PositionControl_OBJECTS = \
"CMakeFiles/PositionControl.dir/PositionControl.cpp.o" \
"CMakeFiles/PositionControl.dir/ControlMath.cpp.o"

# External object files for target PositionControl
PositionControl_EXTERNAL_OBJECTS =

src/modules/mc_pos_control/PositionControl/libPositionControl.a: src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/PositionControl.cpp.o
src/modules/mc_pos_control/PositionControl/libPositionControl.a: src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/ControlMath.cpp.o
src/modules/mc_pos_control/PositionControl/libPositionControl.a: src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/build.make
src/modules/mc_pos_control/PositionControl/libPositionControl.a: src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libPositionControl.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl && $(CMAKE_COMMAND) -P CMakeFiles/PositionControl.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/PositionControl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/build: src/modules/mc_pos_control/PositionControl/libPositionControl.a

.PHONY : src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/build

src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl && $(CMAKE_COMMAND) -P CMakeFiles/PositionControl.dir/cmake_clean.cmake
.PHONY : src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/clean

src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/modules/mc_pos_control/PositionControl /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/mc_pos_control/PositionControl/CMakeFiles/PositionControl.dir/depend

