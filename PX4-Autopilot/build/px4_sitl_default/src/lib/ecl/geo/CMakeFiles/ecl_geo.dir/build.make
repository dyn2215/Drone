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
include src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/depend.make

# Include the progress variables for this target.
include src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/flags.make

src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/geo.cpp.o: src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/flags.make
src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/geo.cpp.o: ../../src/lib/ecl/geo/geo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/geo.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/ecl/geo && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ecl_geo.dir/geo.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/lib/ecl/geo/geo.cpp

src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/geo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ecl_geo.dir/geo.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/ecl/geo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/lib/ecl/geo/geo.cpp > CMakeFiles/ecl_geo.dir/geo.cpp.i

src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/geo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ecl_geo.dir/geo.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/ecl/geo && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/lib/ecl/geo/geo.cpp -o CMakeFiles/ecl_geo.dir/geo.cpp.s

# Object files for target ecl_geo
ecl_geo_OBJECTS = \
"CMakeFiles/ecl_geo.dir/geo.cpp.o"

# External object files for target ecl_geo
ecl_geo_EXTERNAL_OBJECTS =

src/lib/ecl/geo/libecl_geo.a: src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/geo.cpp.o
src/lib/ecl/geo/libecl_geo.a: src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/build.make
src/lib/ecl/geo/libecl_geo.a: src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libecl_geo.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/ecl/geo && $(CMAKE_COMMAND) -P CMakeFiles/ecl_geo.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/ecl/geo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ecl_geo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/build: src/lib/ecl/geo/libecl_geo.a

.PHONY : src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/build

src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/ecl/geo && $(CMAKE_COMMAND) -P CMakeFiles/ecl_geo.dir/cmake_clean.cmake
.PHONY : src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/clean

src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/lib/ecl/geo /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/ecl/geo /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/depend

