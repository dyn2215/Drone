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
include src/lib/slew_rate/CMakeFiles/SlewRate.dir/depend.make

# Include the progress variables for this target.
include src/lib/slew_rate/CMakeFiles/SlewRate.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/slew_rate/CMakeFiles/SlewRate.dir/flags.make

src/lib/slew_rate/CMakeFiles/SlewRate.dir/dummy.cpp.o: src/lib/slew_rate/CMakeFiles/SlewRate.dir/flags.make
src/lib/slew_rate/CMakeFiles/SlewRate.dir/dummy.cpp.o: ../../src/lib/slew_rate/dummy.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/slew_rate/CMakeFiles/SlewRate.dir/dummy.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/slew_rate && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SlewRate.dir/dummy.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/lib/slew_rate/dummy.cpp

src/lib/slew_rate/CMakeFiles/SlewRate.dir/dummy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SlewRate.dir/dummy.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/slew_rate && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/lib/slew_rate/dummy.cpp > CMakeFiles/SlewRate.dir/dummy.cpp.i

src/lib/slew_rate/CMakeFiles/SlewRate.dir/dummy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SlewRate.dir/dummy.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/slew_rate && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/lib/slew_rate/dummy.cpp -o CMakeFiles/SlewRate.dir/dummy.cpp.s

# Object files for target SlewRate
SlewRate_OBJECTS = \
"CMakeFiles/SlewRate.dir/dummy.cpp.o"

# External object files for target SlewRate
SlewRate_EXTERNAL_OBJECTS =

src/lib/slew_rate/libSlewRate.a: src/lib/slew_rate/CMakeFiles/SlewRate.dir/dummy.cpp.o
src/lib/slew_rate/libSlewRate.a: src/lib/slew_rate/CMakeFiles/SlewRate.dir/build.make
src/lib/slew_rate/libSlewRate.a: src/lib/slew_rate/CMakeFiles/SlewRate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libSlewRate.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/slew_rate && $(CMAKE_COMMAND) -P CMakeFiles/SlewRate.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/slew_rate && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SlewRate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/slew_rate/CMakeFiles/SlewRate.dir/build: src/lib/slew_rate/libSlewRate.a

.PHONY : src/lib/slew_rate/CMakeFiles/SlewRate.dir/build

src/lib/slew_rate/CMakeFiles/SlewRate.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/slew_rate && $(CMAKE_COMMAND) -P CMakeFiles/SlewRate.dir/cmake_clean.cmake
.PHONY : src/lib/slew_rate/CMakeFiles/SlewRate.dir/clean

src/lib/slew_rate/CMakeFiles/SlewRate.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/lib/slew_rate /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/slew_rate /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/slew_rate/CMakeFiles/SlewRate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/slew_rate/CMakeFiles/SlewRate.dir/depend

