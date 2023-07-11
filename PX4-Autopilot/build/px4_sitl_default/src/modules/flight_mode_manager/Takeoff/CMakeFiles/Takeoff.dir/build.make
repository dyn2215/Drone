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
include src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/depend.make

# Include the progress variables for this target.
include src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/flags.make

src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/Takeoff.cpp.o: src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/flags.make
src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/Takeoff.cpp.o: ../../src/modules/flight_mode_manager/Takeoff/Takeoff.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/Takeoff.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/flight_mode_manager/Takeoff && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Takeoff.dir/Takeoff.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/flight_mode_manager/Takeoff/Takeoff.cpp

src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/Takeoff.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Takeoff.dir/Takeoff.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/flight_mode_manager/Takeoff && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/flight_mode_manager/Takeoff/Takeoff.cpp > CMakeFiles/Takeoff.dir/Takeoff.cpp.i

src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/Takeoff.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Takeoff.dir/Takeoff.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/flight_mode_manager/Takeoff && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/flight_mode_manager/Takeoff/Takeoff.cpp -o CMakeFiles/Takeoff.dir/Takeoff.cpp.s

# Object files for target Takeoff
Takeoff_OBJECTS = \
"CMakeFiles/Takeoff.dir/Takeoff.cpp.o"

# External object files for target Takeoff
Takeoff_EXTERNAL_OBJECTS =

src/modules/flight_mode_manager/Takeoff/libTakeoff.a: src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/Takeoff.cpp.o
src/modules/flight_mode_manager/Takeoff/libTakeoff.a: src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/build.make
src/modules/flight_mode_manager/Takeoff/libTakeoff.a: src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libTakeoff.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/flight_mode_manager/Takeoff && $(CMAKE_COMMAND) -P CMakeFiles/Takeoff.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/flight_mode_manager/Takeoff && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Takeoff.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/build: src/modules/flight_mode_manager/Takeoff/libTakeoff.a

.PHONY : src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/build

src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/flight_mode_manager/Takeoff && $(CMAKE_COMMAND) -P CMakeFiles/Takeoff.dir/cmake_clean.cmake
.PHONY : src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/clean

src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/modules/flight_mode_manager/Takeoff /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/flight_mode_manager/Takeoff /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/flight_mode_manager/Takeoff/CMakeFiles/Takeoff.dir/depend

