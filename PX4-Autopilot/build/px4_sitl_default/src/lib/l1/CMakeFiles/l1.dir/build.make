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
include src/lib/l1/CMakeFiles/l1.dir/depend.make

# Include the progress variables for this target.
include src/lib/l1/CMakeFiles/l1.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/l1/CMakeFiles/l1.dir/flags.make

src/lib/l1/CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.o: src/lib/l1/CMakeFiles/l1.dir/flags.make
src/lib/l1/CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.o: ../../src/lib/l1/ECL_L1_Pos_Controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/l1/CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/l1 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/lib/l1/ECL_L1_Pos_Controller.cpp

src/lib/l1/CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/l1 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/lib/l1/ECL_L1_Pos_Controller.cpp > CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.i

src/lib/l1/CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/l1 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/lib/l1/ECL_L1_Pos_Controller.cpp -o CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.s

# Object files for target l1
l1_OBJECTS = \
"CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.o"

# External object files for target l1
l1_EXTERNAL_OBJECTS =

src/lib/l1/libl1.a: src/lib/l1/CMakeFiles/l1.dir/ECL_L1_Pos_Controller.cpp.o
src/lib/l1/libl1.a: src/lib/l1/CMakeFiles/l1.dir/build.make
src/lib/l1/libl1.a: src/lib/l1/CMakeFiles/l1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libl1.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/l1 && $(CMAKE_COMMAND) -P CMakeFiles/l1.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/l1 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/l1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/l1/CMakeFiles/l1.dir/build: src/lib/l1/libl1.a

.PHONY : src/lib/l1/CMakeFiles/l1.dir/build

src/lib/l1/CMakeFiles/l1.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/l1 && $(CMAKE_COMMAND) -P CMakeFiles/l1.dir/cmake_clean.cmake
.PHONY : src/lib/l1/CMakeFiles/l1.dir/clean

src/lib/l1/CMakeFiles/l1.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/lib/l1 /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/l1 /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/l1/CMakeFiles/l1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/l1/CMakeFiles/l1.dir/depend

