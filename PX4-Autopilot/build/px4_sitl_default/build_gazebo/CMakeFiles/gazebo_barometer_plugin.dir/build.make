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
CMAKE_SOURCE_DIR = /home/nics/Drone/PX4-Autopilot/Tools/sitl_gazebo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/build_gazebo

# Include any dependencies generated for this target.
include CMakeFiles/gazebo_barometer_plugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gazebo_barometer_plugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gazebo_barometer_plugin.dir/flags.make

CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.o: CMakeFiles/gazebo_barometer_plugin.dir/flags.make
CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.o: /home/nics/Drone/PX4-Autopilot/Tools/sitl_gazebo/src/gazebo_barometer_plugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/build_gazebo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.o -c /home/nics/Drone/PX4-Autopilot/Tools/sitl_gazebo/src/gazebo_barometer_plugin.cpp

CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/Tools/sitl_gazebo/src/gazebo_barometer_plugin.cpp > CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.i

CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/Tools/sitl_gazebo/src/gazebo_barometer_plugin.cpp -o CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.s

# Object files for target gazebo_barometer_plugin
gazebo_barometer_plugin_OBJECTS = \
"CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.o"

# External object files for target gazebo_barometer_plugin
gazebo_barometer_plugin_EXTERNAL_OBJECTS =

libgazebo_barometer_plugin.so: CMakeFiles/gazebo_barometer_plugin.dir/src/gazebo_barometer_plugin.cpp.o
libgazebo_barometer_plugin.so: CMakeFiles/gazebo_barometer_plugin.dir/build.make
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgstreamer-1.0.so
libgazebo_barometer_plugin.so: libmav_msgs.so
libgazebo_barometer_plugin.so: libnav_msgs.so
libgazebo_barometer_plugin.so: libstd_msgs.so
libgazebo_barometer_plugin.so: libsensor_msgs.so
libgazebo_barometer_plugin.so: libphysics_msgs.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libdart.so.6.9.2
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.8.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.14.2
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libdart-external-odelcpsolver.so.6.9.2
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libccd.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libfcl.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libassimp.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.3
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/liboctomath.so.1.9.3
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.3.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.6.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.10.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.13.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.14.2
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_gapi.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_stitching.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_alphamat.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_aruco.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_barcode.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_bgsegm.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_bioinspired.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_ccalib.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_dnn_objdetect.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_dnn_superres.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_dpm.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_face.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_fuzzy.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_hfs.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_img_hash.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_intensity_transform.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_line_descriptor.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_mcc.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_quality.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_rapid.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_reg.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_rgbd.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_saliency.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_stereo.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_structured_light.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_phase_unwrapping.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_superres.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_optflow.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_surface_matching.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_tracking.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_highgui.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_datasets.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_plot.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_text.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_videostab.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_videoio.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_wechat_qrcode.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_xfeatures2d.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_ml.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_shape.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_ximgproc.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_video.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_xobjdetect.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_imgcodecs.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_objdetect.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_calib3d.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_dnn.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_features2d.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_flann.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_xphoto.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_photo.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_imgproc.so.4.6.0
libgazebo_barometer_plugin.so: /usr/local/lib/libopencv_core.so.4.6.0
libgazebo_barometer_plugin.so: /usr/lib/x86_64-linux-gnu/libgstreamer-1.0.so
libgazebo_barometer_plugin.so: CMakeFiles/gazebo_barometer_plugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/build_gazebo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libgazebo_barometer_plugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gazebo_barometer_plugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gazebo_barometer_plugin.dir/build: libgazebo_barometer_plugin.so

.PHONY : CMakeFiles/gazebo_barometer_plugin.dir/build

CMakeFiles/gazebo_barometer_plugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gazebo_barometer_plugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gazebo_barometer_plugin.dir/clean

CMakeFiles/gazebo_barometer_plugin.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/build_gazebo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot/Tools/sitl_gazebo /home/nics/Drone/PX4-Autopilot/Tools/sitl_gazebo /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/build_gazebo /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/build_gazebo /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/build_gazebo/CMakeFiles/gazebo_barometer_plugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gazebo_barometer_plugin.dir/depend

