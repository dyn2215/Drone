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
include src/lib/perf/CMakeFiles/perf.dir/depend.make

# Include the progress variables for this target.
include src/lib/perf/CMakeFiles/perf.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/perf/CMakeFiles/perf.dir/flags.make

src/lib/perf/CMakeFiles/perf.dir/perf_counter.cpp.o: src/lib/perf/CMakeFiles/perf.dir/flags.make
src/lib/perf/CMakeFiles/perf.dir/perf_counter.cpp.o: ../../src/lib/perf/perf_counter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/perf/CMakeFiles/perf.dir/perf_counter.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/perf && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/perf.dir/perf_counter.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/lib/perf/perf_counter.cpp

src/lib/perf/CMakeFiles/perf.dir/perf_counter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/perf.dir/perf_counter.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/perf && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/lib/perf/perf_counter.cpp > CMakeFiles/perf.dir/perf_counter.cpp.i

src/lib/perf/CMakeFiles/perf.dir/perf_counter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/perf.dir/perf_counter.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/perf && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/lib/perf/perf_counter.cpp -o CMakeFiles/perf.dir/perf_counter.cpp.s

# Object files for target perf
perf_OBJECTS = \
"CMakeFiles/perf.dir/perf_counter.cpp.o"

# External object files for target perf
perf_EXTERNAL_OBJECTS =

src/lib/perf/libperf.a: src/lib/perf/CMakeFiles/perf.dir/perf_counter.cpp.o
src/lib/perf/libperf.a: src/lib/perf/CMakeFiles/perf.dir/build.make
src/lib/perf/libperf.a: src/lib/perf/CMakeFiles/perf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libperf.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/perf && $(CMAKE_COMMAND) -P CMakeFiles/perf.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/perf && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/perf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/perf/CMakeFiles/perf.dir/build: src/lib/perf/libperf.a

.PHONY : src/lib/perf/CMakeFiles/perf.dir/build

src/lib/perf/CMakeFiles/perf.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/perf && $(CMAKE_COMMAND) -P CMakeFiles/perf.dir/cmake_clean.cmake
.PHONY : src/lib/perf/CMakeFiles/perf.dir/clean

src/lib/perf/CMakeFiles/perf.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/lib/perf /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/perf /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/perf/CMakeFiles/perf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/perf/CMakeFiles/perf.dir/depend

