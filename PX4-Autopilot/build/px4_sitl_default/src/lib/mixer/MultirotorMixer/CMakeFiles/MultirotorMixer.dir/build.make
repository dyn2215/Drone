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
include src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/depend.make

# Include the progress variables for this target.
include src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/progress.make

# Include the compile flags for this target's objects.
include src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/flags.make

src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/tools/px_generate_mixers.py
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/dodeca_bottom_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/dodeca_top_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_plus.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_t.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_x.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_cox_wide.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_plus.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_x.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_deadcat.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_h.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_plus.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_s250aq.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_vtail.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_wide.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_x_cw.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_x.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_x_pusher.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_y.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/tri_y.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/twin_engine.toml
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating mixer_multirotor.generated.h"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer && /usr/bin/python3 /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/tools/px_generate_mixers.py -f /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/dodeca_bottom_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/dodeca_top_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_plus.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_t.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_x.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_cox_wide.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_plus.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_x.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_deadcat.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_h.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_plus.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_s250aq.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_vtail.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_wide.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_x_cw.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_x.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_x_pusher.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_y.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/tri_y.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/twin_engine.toml -o mixer_multirotor.generated.h

src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/tools/px_generate_mixers.py
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/dodeca_bottom_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/dodeca_top_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_plus.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_t.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_x.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_cox_wide.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_plus.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_x.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_deadcat.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_h.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_plus.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_s250aq.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_vtail.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_wide.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_x_cw.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_x.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_x_pusher.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_y.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/tri_y.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/twin_engine.toml
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating mixer_multirotor_normalized.generated.h"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer && /usr/bin/python3 /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/tools/px_generate_mixers.py --normalize -f /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/dodeca_bottom_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/dodeca_top_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_plus.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_t.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_x.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_cox_wide.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_plus.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_x.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_deadcat.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_h.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_plus.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_s250aq.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_vtail.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_wide.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_x_cw.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_x.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_x_pusher.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_y.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/tri_y.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/twin_engine.toml -o mixer_multirotor_normalized.generated.h

src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/tools/px_generate_mixers.py
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/dodeca_bottom_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/dodeca_top_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_plus.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_t.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/hex_x.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_cox.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_cox_wide.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_plus.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/octa_x.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_deadcat.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_h.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_plus.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_s250aq.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_vtail.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_wide.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_x_cw.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_x.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_x_pusher.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/quad_y.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/tri_y.toml
src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h: ../../src/lib/mixer/MultirotorMixer/geometries/twin_engine.toml
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating mixer_multirotor_6dof.generated.h"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer && /usr/bin/python3 /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/tools/px_generate_mixers.py --sixdof -f /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/dodeca_bottom_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/dodeca_top_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_plus.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_t.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/hex_x.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_cox.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_cox_wide.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_plus.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/octa_x.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_deadcat.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_h.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_plus.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_s250aq.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_vtail.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_wide.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_x_cw.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_x.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_x_pusher.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/quad_y.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/tri_y.toml /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/geometries/twin_engine.toml -o mixer_multirotor_6dof.generated.h

src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.o: src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/flags.make
src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.o: ../../src/lib/mixer/MultirotorMixer/MultirotorMixer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.o -c /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/MultirotorMixer.cpp

src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/MultirotorMixer.cpp > CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.i

src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer/MultirotorMixer.cpp -o CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.s

# Object files for target MultirotorMixer
MultirotorMixer_OBJECTS = \
"CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.o"

# External object files for target MultirotorMixer
MultirotorMixer_EXTERNAL_OBJECTS =

src/lib/mixer/MultirotorMixer/libMultirotorMixer.a: src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/MultirotorMixer.cpp.o
src/lib/mixer/MultirotorMixer/libMultirotorMixer.a: src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/build.make
src/lib/mixer/MultirotorMixer/libMultirotorMixer.a: src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library libMultirotorMixer.a"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer && $(CMAKE_COMMAND) -P CMakeFiles/MultirotorMixer.dir/cmake_clean_target.cmake
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MultirotorMixer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/build: src/lib/mixer/MultirotorMixer/libMultirotorMixer.a

.PHONY : src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/build

src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer && $(CMAKE_COMMAND) -P CMakeFiles/MultirotorMixer.dir/cmake_clean.cmake
.PHONY : src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/clean

src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/depend: src/lib/mixer/MultirotorMixer/mixer_multirotor.generated.h
src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/depend: src/lib/mixer/MultirotorMixer/mixer_multirotor_normalized.generated.h
src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/depend: src/lib/mixer/MultirotorMixer/mixer_multirotor_6dof.generated.h
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/src/lib/mixer/MultirotorMixer /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/lib/mixer/MultirotorMixer/CMakeFiles/MultirotorMixer.dir/depend

