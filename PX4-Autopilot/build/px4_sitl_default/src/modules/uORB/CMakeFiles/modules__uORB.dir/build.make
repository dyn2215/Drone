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
include src/modules/uORB/CMakeFiles/modules__uORB.dir/depend.make

# Include the progress variables for this target.
include src/modules/uORB/CMakeFiles/modules__uORB.dir/progress.make

# Include the compile flags for this target's objects.
include src/modules/uORB/CMakeFiles/modules__uORB.dir/flags.make

src/modules/uORB/CMakeFiles/modules__uORB.dir/Subscription.cpp.o: src/modules/uORB/CMakeFiles/modules__uORB.dir/flags.make
src/modules/uORB/CMakeFiles/modules__uORB.dir/Subscription.cpp.o: ../../src/modules/uORB/Subscription.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/modules/uORB/CMakeFiles/modules__uORB.dir/Subscription.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__uORB.dir/Subscription.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/uORB/Subscription.cpp

src/modules/uORB/CMakeFiles/modules__uORB.dir/Subscription.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__uORB.dir/Subscription.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/uORB/Subscription.cpp > CMakeFiles/modules__uORB.dir/Subscription.cpp.i

src/modules/uORB/CMakeFiles/modules__uORB.dir/Subscription.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__uORB.dir/Subscription.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/uORB/Subscription.cpp -o CMakeFiles/modules__uORB.dir/Subscription.cpp.s

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORB.cpp.o: src/modules/uORB/CMakeFiles/modules__uORB.dir/flags.make
src/modules/uORB/CMakeFiles/modules__uORB.dir/uORB.cpp.o: ../../src/modules/uORB/uORB.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/modules/uORB/CMakeFiles/modules__uORB.dir/uORB.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__uORB.dir/uORB.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORB.cpp

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__uORB.dir/uORB.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORB.cpp > CMakeFiles/modules__uORB.dir/uORB.cpp.i

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__uORB.dir/uORB.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORB.cpp -o CMakeFiles/modules__uORB.dir/uORB.cpp.s

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.o: src/modules/uORB/CMakeFiles/modules__uORB.dir/flags.make
src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.o: ../../src/modules/uORB/uORBDeviceMaster.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBDeviceMaster.cpp

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBDeviceMaster.cpp > CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.i

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBDeviceMaster.cpp -o CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.s

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.o: src/modules/uORB/CMakeFiles/modules__uORB.dir/flags.make
src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.o: ../../src/modules/uORB/uORBDeviceNode.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBDeviceNode.cpp

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBDeviceNode.cpp > CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.i

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBDeviceNode.cpp -o CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.s

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBMain.cpp.o: src/modules/uORB/CMakeFiles/modules__uORB.dir/flags.make
src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBMain.cpp.o: ../../src/modules/uORB/uORBMain.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBMain.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__uORB.dir/uORBMain.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBMain.cpp

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBMain.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__uORB.dir/uORBMain.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBMain.cpp > CMakeFiles/modules__uORB.dir/uORBMain.cpp.i

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBMain.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__uORB.dir/uORBMain.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBMain.cpp -o CMakeFiles/modules__uORB.dir/uORBMain.cpp.s

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBManager.cpp.o: src/modules/uORB/CMakeFiles/modules__uORB.dir/flags.make
src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBManager.cpp.o: ../../src/modules/uORB/uORBManager.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBManager.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__uORB.dir/uORBManager.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBManager.cpp

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBManager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__uORB.dir/uORBManager.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBManager.cpp > CMakeFiles/modules__uORB.dir/uORBManager.cpp.i

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBManager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__uORB.dir/uORBManager.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBManager.cpp -o CMakeFiles/modules__uORB.dir/uORBManager.cpp.s

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBUtils.cpp.o: src/modules/uORB/CMakeFiles/modules__uORB.dir/flags.make
src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBUtils.cpp.o: ../../src/modules/uORB/uORBUtils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBUtils.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/modules__uORB.dir/uORBUtils.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBUtils.cpp

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBUtils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/modules__uORB.dir/uORBUtils.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBUtils.cpp > CMakeFiles/modules__uORB.dir/uORBUtils.cpp.i

src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBUtils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/modules__uORB.dir/uORBUtils.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/modules/uORB/uORBUtils.cpp -o CMakeFiles/modules__uORB.dir/uORBUtils.cpp.s

# Object files for target modules__uORB
modules__uORB_OBJECTS = \
"CMakeFiles/modules__uORB.dir/Subscription.cpp.o" \
"CMakeFiles/modules__uORB.dir/uORB.cpp.o" \
"CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.o" \
"CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.o" \
"CMakeFiles/modules__uORB.dir/uORBMain.cpp.o" \
"CMakeFiles/modules__uORB.dir/uORBManager.cpp.o" \
"CMakeFiles/modules__uORB.dir/uORBUtils.cpp.o"

# External object files for target modules__uORB
modules__uORB_EXTERNAL_OBJECTS =

src/modules/uORB/libmodules__uORB.a: src/modules/uORB/CMakeFiles/modules__uORB.dir/Subscription.cpp.o
src/modules/uORB/libmodules__uORB.a: src/modules/uORB/CMakeFiles/modules__uORB.dir/uORB.cpp.o
src/modules/uORB/libmodules__uORB.a: src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceMaster.cpp.o
src/modules/uORB/libmodules__uORB.a: src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBDeviceNode.cpp.o
src/modules/uORB/libmodules__uORB.a: src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBMain.cpp.o
src/modules/uORB/libmodules__uORB.a: src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBManager.cpp.o
src/modules/uORB/libmodules__uORB.a: src/modules/uORB/CMakeFiles/modules__uORB.dir/uORBUtils.cpp.o
src/modules/uORB/libmodules__uORB.a: src/modules/uORB/CMakeFiles/modules__uORB.dir/build.make
src/modules/uORB/libmodules__uORB.a: src/modules/uORB/CMakeFiles/modules__uORB.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX static library libmodules__uORB.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && $(CMAKE_COMMAND) -P CMakeFiles/modules__uORB.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/modules__uORB.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/modules/uORB/CMakeFiles/modules__uORB.dir/build: src/modules/uORB/libmodules__uORB.a

.PHONY : src/modules/uORB/CMakeFiles/modules__uORB.dir/build

src/modules/uORB/CMakeFiles/modules__uORB.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB && $(CMAKE_COMMAND) -P CMakeFiles/modules__uORB.dir/cmake_clean.cmake
.PHONY : src/modules/uORB/CMakeFiles/modules__uORB.dir/clean

src/modules/uORB/CMakeFiles/modules__uORB.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/modules/uORB /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB/CMakeFiles/modules__uORB.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/modules/uORB/CMakeFiles/modules__uORB.dir/depend

