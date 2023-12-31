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
include src/lib/battery/CMakeFiles/battery.dir/depend.make

# Include the progress variables for this target.
include src/lib/battery/CMakeFiles/battery.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/battery/CMakeFiles/battery.dir/flags.make

src/lib/battery/CMakeFiles/battery.dir/battery.cpp.o: src/lib/battery/CMakeFiles/battery.dir/flags.make
src/lib/battery/CMakeFiles/battery.dir/battery.cpp.o: ../../src/lib/battery/battery.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/battery/CMakeFiles/battery.dir/battery.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/battery && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/battery.dir/battery.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/lib/battery/battery.cpp

src/lib/battery/CMakeFiles/battery.dir/battery.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/battery.dir/battery.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/battery && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/lib/battery/battery.cpp > CMakeFiles/battery.dir/battery.cpp.i

src/lib/battery/CMakeFiles/battery.dir/battery.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/battery.dir/battery.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/battery && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/lib/battery/battery.cpp -o CMakeFiles/battery.dir/battery.cpp.s

# Object files for target battery
battery_OBJECTS = \
"CMakeFiles/battery.dir/battery.cpp.o"

# External object files for target battery
battery_EXTERNAL_OBJECTS =

src/lib/battery/libbattery.a: src/lib/battery/CMakeFiles/battery.dir/battery.cpp.o
src/lib/battery/libbattery.a: src/lib/battery/CMakeFiles/battery.dir/build.make
src/lib/battery/libbattery.a: src/lib/battery/CMakeFiles/battery.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libbattery.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/battery && $(CMAKE_COMMAND) -P CMakeFiles/battery.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/battery && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/battery.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/battery/CMakeFiles/battery.dir/build: src/lib/battery/libbattery.a

.PHONY : src/lib/battery/CMakeFiles/battery.dir/build

src/lib/battery/CMakeFiles/battery.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/battery && $(CMAKE_COMMAND) -P CMakeFiles/battery.dir/cmake_clean.cmake
.PHONY : src/lib/battery/CMakeFiles/battery.dir/clean

src/lib/battery/CMakeFiles/battery.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/lib/battery /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/battery /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/battery/CMakeFiles/battery.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/battery/CMakeFiles/battery.dir/depend

