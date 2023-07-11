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
include src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/depend.make

# Include the progress variables for this target.
include src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/flags.make

src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.o: src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/flags.make
src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.o: ../../src/lib/magnetometer_bias_estimator/MagnetometerBiasEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/magnetometer_bias_estimator && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/lib/magnetometer_bias_estimator/MagnetometerBiasEstimator.cpp

src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/magnetometer_bias_estimator && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/lib/magnetometer_bias_estimator/MagnetometerBiasEstimator.cpp > CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.i

src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/magnetometer_bias_estimator && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/lib/magnetometer_bias_estimator/MagnetometerBiasEstimator.cpp -o CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.s

# Object files for target MagnetometerBiasEstimator
MagnetometerBiasEstimator_OBJECTS = \
"CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.o"

# External object files for target MagnetometerBiasEstimator
MagnetometerBiasEstimator_EXTERNAL_OBJECTS =

src/lib/magnetometer_bias_estimator/libMagnetometerBiasEstimator.a: src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/MagnetometerBiasEstimator.cpp.o
src/lib/magnetometer_bias_estimator/libMagnetometerBiasEstimator.a: src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/build.make
src/lib/magnetometer_bias_estimator/libMagnetometerBiasEstimator.a: src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libMagnetometerBiasEstimator.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/magnetometer_bias_estimator && $(CMAKE_COMMAND) -P CMakeFiles/MagnetometerBiasEstimator.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/magnetometer_bias_estimator && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MagnetometerBiasEstimator.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/build: src/lib/magnetometer_bias_estimator/libMagnetometerBiasEstimator.a

.PHONY : src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/build

src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/magnetometer_bias_estimator && $(CMAKE_COMMAND) -P CMakeFiles/MagnetometerBiasEstimator.dir/cmake_clean.cmake
.PHONY : src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/clean

src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/lib/magnetometer_bias_estimator /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/magnetometer_bias_estimator /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/magnetometer_bias_estimator/CMakeFiles/MagnetometerBiasEstimator.dir/depend
