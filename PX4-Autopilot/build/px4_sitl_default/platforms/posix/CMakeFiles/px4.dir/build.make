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
include platforms/posix/CMakeFiles/px4.dir/depend.make

# Include the progress variables for this target.
include platforms/posix/CMakeFiles/px4.dir/progress.make

# Include the compile flags for this target's objects.
include platforms/posix/CMakeFiles/px4.dir/flags.make

platforms/posix/CMakeFiles/px4.dir/src/px4/common/main.cpp.o: platforms/posix/CMakeFiles/px4.dir/flags.make
platforms/posix/CMakeFiles/px4.dir/src/px4/common/main.cpp.o: ../../platforms/posix/src/px4/common/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object platforms/posix/CMakeFiles/px4.dir/src/px4/common/main.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/px4.dir/src/px4/common/main.cpp.o -c /home/nics/Drone/PX4-Autopilot/platforms/posix/src/px4/common/main.cpp

platforms/posix/CMakeFiles/px4.dir/src/px4/common/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/px4.dir/src/px4/common/main.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/platforms/posix/src/px4/common/main.cpp > CMakeFiles/px4.dir/src/px4/common/main.cpp.i

platforms/posix/CMakeFiles/px4.dir/src/px4/common/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/px4.dir/src/px4/common/main.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/platforms/posix/src/px4/common/main.cpp -o CMakeFiles/px4.dir/src/px4/common/main.cpp.s

platforms/posix/CMakeFiles/px4.dir/apps.cpp.o: platforms/posix/CMakeFiles/px4.dir/flags.make
platforms/posix/CMakeFiles/px4.dir/apps.cpp.o: platforms/posix/apps.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object platforms/posix/CMakeFiles/px4.dir/apps.cpp.o"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/px4.dir/apps.cpp.o -c /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/apps.cpp

platforms/posix/CMakeFiles/px4.dir/apps.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/px4.dir/apps.cpp.i"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/apps.cpp > CMakeFiles/px4.dir/apps.cpp.i

platforms/posix/CMakeFiles/px4.dir/apps.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/px4.dir/apps.cpp.s"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/apps.cpp -o CMakeFiles/px4.dir/apps.cpp.s

# Object files for target px4
px4_OBJECTS = \
"CMakeFiles/px4.dir/src/px4/common/main.cpp.o" \
"CMakeFiles/px4.dir/apps.cpp.o"

# External object files for target px4
px4_EXTERNAL_OBJECTS =

bin/px4: platforms/posix/CMakeFiles/px4.dir/src/px4/common/main.cpp.o
bin/px4: platforms/posix/CMakeFiles/px4.dir/apps.cpp.o
bin/px4: platforms/posix/CMakeFiles/px4.dir/build.make
bin/px4: src/lib/cdev/test/liblib__cdev__test__cdev_test.a
bin/px4: src/lib/controllib/controllib_test/liblib__controllib__controllib_test.a
bin/px4: src/lib/rc/rc_tests/liblib__rc__rc_tests.a
bin/px4: platforms/common/px4_work_queue/test/liblib__work_queue__test__wqueue_test.a
bin/px4: src/modules/uORB/libmodules__uORB.a
bin/px4: src/modules/uORB/uORB_tests/libmodules__uORB__uORB_tests.a
bin/px4: src/drivers/camera_capture/libdrivers__camera_capture.a
bin/px4: src/drivers/camera_trigger/libdrivers__camera_trigger.a
bin/px4: src/drivers/gps/libdrivers__gps.a
bin/px4: src/drivers/pwm_out_sim/libdrivers__pwm_out_sim.a
bin/px4: src/drivers/rpm/rpm_simulator/libexamples__rpm_simulator.a
bin/px4: src/drivers/tone_alarm/libdrivers__tone_alarm.a
bin/px4: src/modules/airship_att_control/libmodules__airship_att_control.a
bin/px4: src/modules/airspeed_selector/libmodules__airspeed_selector.a
bin/px4: src/modules/attitude_estimator_q/libmodules__attitude_estimator_q.a
bin/px4: src/modules/camera_feedback/libmodules__camera_feedback.a
bin/px4: src/modules/commander/libmodules__commander.a
bin/px4: src/modules/commander/commander_tests/libmodules__commander__commander_tests.a
bin/px4: src/modules/dataman/libmodules__dataman.a
bin/px4: src/modules/ekf2/libmodules__ekf2.a
bin/px4: src/modules/events/libmodules__events.a
bin/px4: src/modules/flight_mode_manager/libmodules__flight_mode_manager.a
bin/px4: src/modules/fw_att_control/libmodules__fw_att_control.a
bin/px4: src/modules/fw_pos_control_l1/libmodules__fw_pos_control_l1.a
bin/px4: src/modules/land_detector/libmodules__land_detector.a
bin/px4: src/modules/landing_target_estimator/libmodules__landing_target_estimator.a
bin/px4: src/modules/load_mon/libmodules__load_mon.a
bin/px4: src/modules/local_position_estimator/libmodules__local_position_estimator.a
bin/px4: src/modules/logger/libmodules__logger.a
bin/px4: src/modules/mavlink/libmodules__mavlink.a
bin/px4: src/modules/mavlink/mavlink_tests/libmodules__mavlink__mavlink_tests.a
bin/px4: src/modules/mc_att_control/libmodules__mc_att_control.a
bin/px4: src/modules/mc_hover_thrust_estimator/libmodules__mc_hover_thrust_estimator.a
bin/px4: src/modules/mc_pos_control/libmodules__mc_pos_control.a
bin/px4: src/modules/mc_rate_control/libmodules__mc_rate_control.a
bin/px4: src/modules/navigator/libmodules__navigator.a
bin/px4: src/modules/rc_update/libmodules__rc_update.a
bin/px4: src/modules/replay/libmodules__replay.a
bin/px4: src/modules/rover_pos_control/libmodules__rover_pos_control.a
bin/px4: src/modules/sensors/libmodules__sensors.a
bin/px4: src/modules/simulator/libmodules__simulator.a
bin/px4: src/modules/simulator/battery_simulator/libmodules__simulator__battery_simulator.a
bin/px4: src/modules/temperature_compensation/libmodules__temperature_compensation.a
bin/px4: src/modules/uuv_att_control/libmodules__uuv_att_control.a
bin/px4: src/modules/vmount/libdrivers__vmount.a
bin/px4: src/modules/vtol_att_control/libmodules__vtol_att_control.a
bin/px4: src/systemcmds/dyn/libsystemcmds__dyn.a
bin/px4: src/systemcmds/esc_calib/libsystemcmds__esc_calib.a
bin/px4: src/systemcmds/failure/libsystemcmds__failure.a
bin/px4: src/systemcmds/led_control/libsystemcmds__led_control.a
bin/px4: src/systemcmds/mixer/libsystemcmds__mixer.a
bin/px4: src/systemcmds/motor_ramp/libsystemcmds__motor_ramp.a
bin/px4: src/systemcmds/motor_test/libsystemcmds__motor_test.a
bin/px4: src/systemcmds/param/libsystemcmds__param.a
bin/px4: src/systemcmds/perf/libsystemcmds__perf.a
bin/px4: src/systemcmds/pwm/libsystemcmds__pwm.a
bin/px4: src/systemcmds/sd_bench/libsystemcmds__sd_bench.a
bin/px4: src/systemcmds/shutdown/libsystemcmds__shutdown.a
bin/px4: src/systemcmds/system_time/libsystemcmds__system_time.a
bin/px4: src/systemcmds/tests/libsystemcmds__tests.a
bin/px4: src/systemcmds/tests/hrt_test/libsystemcmds__tests__hrt_test.a
bin/px4: src/systemcmds/topic_listener/libsystemcmds__topic_listener.a
bin/px4: src/systemcmds/tune_control/libsystemcmds__tune_control.a
bin/px4: src/systemcmds/ver/libsystemcmds__ver.a
bin/px4: src/systemcmds/work_queue/libsystemcmds__work_queue.a
bin/px4: src/examples/fake_magnetometer/libexamples__fake_magnetometer.a
bin/px4: src/examples/fixedwing_control/libexamples__fixedwing_control.a
bin/px4: src/examples/hello/libexamples__hello.a
bin/px4: src/examples/px4_mavlink_debug/libexamples__px4_mavlink_debug.a
bin/px4: src/examples/px4_simple_app/libexamples__px4_simple_app.a
bin/px4: src/examples/rover_steering_control/libexamples__rover_steering_control.a
bin/px4: src/examples/uuv_example_app/libexamples__uuv_example_app.a
bin/px4: src/examples/work_item/libexamples__work_item.a
bin/px4: platforms/posix/src/px4/common/libpx4_layer.a
bin/px4: platforms/common/libpx4_platform.a
bin/px4: platforms/common/work_queue/libwork_queue.a
bin/px4: src/lib/parameters/libparameters.a
bin/px4: src/modules/uORB/libmodules__uORB.a
bin/px4: src/lib/rc/librc.a
bin/px4: src/lib/mixer_module/libmixer_module.a
bin/px4: platforms/posix/src/px4/generic/generic/tone_alarm/libarch_tone_alarm.a
bin/px4: src/lib/airspeed_validator/libAirspeedValidator.a
bin/px4: src/lib/ecl/airdata/libecl_airdata.a
bin/px4: src/modules/commander/failure_detector/libfailure_detector.a
bin/px4: src/modules/commander/Arming/PreFlightCheck/libPreFlightCheck.a
bin/px4: src/modules/commander/Arming/ArmAuthorization/libArmAuthorization.a
bin/px4: src/modules/commander/Arming/HealthFlags/libHealthFlags.a
bin/px4: src/lib/ecl/EKF/libecl_EKF.a
bin/px4: src/modules/ekf2/Utility/libEKF2Utility.a
bin/px4: src/modules/flight_mode_manager/Takeoff/libTakeoff.a
bin/px4: src/lib/weather_vane/libWeatherVane.a
bin/px4: src/modules/flight_mode_manager/tasks/ManualPositionSmoothVel/libFlightTaskManualPositionSmoothVel.a
bin/px4: src/modules/flight_mode_manager/tasks/ManualPosition/libFlightTaskManualPosition.a
bin/px4: src/lib/collision_prevention/libCollisionPrevention.a
bin/px4: src/modules/flight_mode_manager/tasks/AutoLineSmoothVel/libFlightTaskAutoLineSmoothVel.a
bin/px4: src/modules/flight_mode_manager/tasks/AutoMapper/libFlightTaskAutoMapper.a
bin/px4: src/modules/flight_mode_manager/tasks/AutoFollowMe/libFlightTaskAutoFollowMe.a
bin/px4: src/modules/flight_mode_manager/tasks/Auto/libFlightTaskAuto.a
bin/px4: src/lib/avoidance/libavoidance.a
bin/px4: src/modules/flight_mode_manager/tasks/Offboard/libFlightTaskOffboard.a
bin/px4: src/modules/flight_mode_manager/tasks/Failsafe/libFlightTaskFailsafe.a
bin/px4: src/modules/flight_mode_manager/tasks/Descend/libFlightTaskDescend.a
bin/px4: src/modules/flight_mode_manager/tasks/Transition/libFlightTaskTransition.a
bin/px4: src/modules/flight_mode_manager/tasks/ManualAcceleration/libFlightTaskManualAcceleration.a
bin/px4: src/modules/flight_mode_manager/tasks/Orbit/libFlightTaskOrbit.a
bin/px4: src/modules/flight_mode_manager/tasks/ManualAltitudeSmoothVel/libFlightTaskManualAltitudeSmoothVel.a
bin/px4: src/modules/flight_mode_manager/tasks/ManualAltitude/libFlightTaskManualAltitude.a
bin/px4: src/modules/flight_mode_manager/tasks/Utility/libFlightTaskUtility.a
bin/px4: src/modules/flight_mode_manager/tasks/FlightTask/libFlightTask.a
bin/px4: src/lib/bezier/libbezier.a
bin/px4: src/lib/slew_rate/libSlewRate.a
bin/px4: src/modules/fw_pos_control_l1/launchdetection/liblaunchdetection.a
bin/px4: src/modules/fw_pos_control_l1/runway_takeoff/librunway_takeoff.a
bin/px4: src/lib/tecs/libtecs.a
bin/px4: src/modules/mc_att_control/AttitudeControl/libAttitudeControl.a
bin/px4: src/lib/hysteresis/libhysteresis.a
bin/px4: src/modules/mc_hover_thrust_estimator/libzero_order_hover_thrust_ekf.a
bin/px4: src/lib/controllib/libcontrollib.a
bin/px4: src/modules/mc_pos_control/PositionControl/libPositionControl.a
bin/px4: src/lib/circuit_breaker/libcircuit_breaker.a
bin/px4: src/modules/mc_rate_control/RateControl/libRateControl.a
bin/px4: src/lib/landing_slope/liblanding_slope.a
bin/px4: src/modules/navigator/GeofenceBreachAvoidance/libgeofence_breach_avoidance.a
bin/px4: src/lib/motion_planning/libmotion_planning.a
bin/px4: src/lib/l1/libl1.a
bin/px4: src/lib/pid/libpid.a
bin/px4: src/lib/airspeed/libairspeed.a
bin/px4: src/modules/sensors/data_validator/libdata_validator.a
bin/px4: src/modules/sensors/vehicle_acceleration/libvehicle_acceleration.a
bin/px4: src/modules/sensors/vehicle_angular_velocity/libvehicle_angular_velocity.a
bin/px4: src/modules/sensors/vehicle_air_data/libvehicle_air_data.a
bin/px4: src/modules/sensors/vehicle_gps_position/libvehicle_gps_position.a
bin/px4: src/modules/sensors/vehicle_imu/libvehicle_imu.a
bin/px4: src/modules/sensors/vehicle_magnetometer/libvehicle_magnetometer.a
bin/px4: src/lib/sensor_calibration/libsensor_calibration.a
bin/px4: src/lib/drivers/accelerometer/libdrivers_accelerometer.a
bin/px4: src/lib/drivers/barometer/libdrivers_barometer.a
bin/px4: src/lib/drivers/gyroscope/libdrivers_gyroscope.a
bin/px4: src/lib/drivers/device/libdrivers__device.a
bin/px4: src/lib/battery/libbattery.a
bin/px4: src/lib/mathlib/libmathlib.a
bin/px4: src/lib/mixer/libmixer.a
bin/px4: src/lib/mixer/AllocatedActuatorMixer/libAllocatedActuatorMixer.a
bin/px4: src/lib/mixer/HelicopterMixer/libHelicopterMixer.a
bin/px4: src/lib/mixer/MultirotorMixer/libMultirotorMixer.a
bin/px4: src/lib/mixer/NullMixer/libNullMixer.a
bin/px4: src/lib/mixer/SimpleMixer/libSimpleMixer.a
bin/px4: src/lib/mixer/MixerBase/libMixerBase.a
bin/px4: src/lib/output_limit/liboutput_limit.a
bin/px4: src/lib/ecl/geo_lookup/libecl_geo_lookup.a
bin/px4: src/lib/tunes/libtunes.a
bin/px4: src/lib/version/libversion.a
bin/px4: src/lib/ecl/geo/libecl_geo.a
bin/px4: src/lib/drivers/magnetometer/libdrivers_magnetometer.a
bin/px4: src/lib/conversion/libconversion.a
bin/px4: src/modules/uORB/libmodules__uORB.a
bin/px4: platforms/posix/src/px4/common/libpx4_layer.a
bin/px4: platforms/common/libpx4_platform.a
bin/px4: src/lib/systemlib/libsystemlib.a
bin/px4: src/lib/cdev/libcdev.a
bin/px4: platforms/common/px4_work_queue/libpx4_work_queue.a
bin/px4: platforms/posix/src/px4/common/px4_daemon/libpx4_daemon.a
bin/px4: platforms/posix/src/px4/common/lockstep_scheduler/liblockstep_scheduler.a
bin/px4: src/modules/uORB/libmodules__uORB.a
bin/px4: platforms/posix/src/px4/common/libpx4_layer.a
bin/px4: platforms/common/libpx4_platform.a
bin/px4: src/lib/systemlib/libsystemlib.a
bin/px4: src/lib/cdev/libcdev.a
bin/px4: platforms/common/px4_work_queue/libpx4_work_queue.a
bin/px4: platforms/posix/src/px4/common/px4_daemon/libpx4_daemon.a
bin/px4: platforms/posix/src/px4/common/lockstep_scheduler/liblockstep_scheduler.a
bin/px4: platforms/common/work_queue/libwork_queue.a
bin/px4: boards/px4/sitl/src/libdrivers_board.a
bin/px4: src/lib/parameters/libparameters.a
bin/px4: src/lib/perf/libperf.a
bin/px4: src/lib/parameters/tinybson/libtinybson.a
bin/px4: msg/libuorb_msgs.a
bin/px4: platforms/posix/CMakeFiles/px4.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../bin/px4"
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/px4.dir/link.txt --verbose=$(VERBOSE)
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-cdev_test
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-controllib_test
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-rc_tests
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-wqueue_test
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-uorb
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-uorb_tests
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-camera_capture
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-camera_trigger
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-gps
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-pwm_out_sim
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-rpm_simulator
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-tone_alarm
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-airship_att_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-airspeed_selector
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-attitude_estimator_q
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-camera_feedback
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-commander
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-commander_tests
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-dataman
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-ekf2
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-send_event
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-flight_mode_manager
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-fw_att_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-fw_pos_control_l1
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-land_detector
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-landing_target_estimator
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-load_mon
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-local_position_estimator
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-logger
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-mavlink
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-mavlink_tests
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-mc_att_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-mc_hover_thrust_estimator
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-mc_pos_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-mc_rate_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-navigator
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-rc_update
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-replay
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-rover_pos_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-sensors
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-simulator
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-battery_simulator
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-temperature_compensation
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-uuv_att_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-vmount
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-vtol_att_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-dyn
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-esc_calib
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-failure
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-led_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-mixer
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-motor_ramp
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-motor_test
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-param
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-perf
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-pwm
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-sd_bench
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-shutdown
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-system_time
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-tests
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-hrt_test
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-listener
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-tune_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-ver
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-work_queue
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-fake_magnetometer
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-ex_fixedwing_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-hello
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-px4_mavlink_debug
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-px4_simple_app
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-rover_steering_control
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-uuv_example_app
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/bin && /usr/bin/cmake -E create_symlink px4 px4-work_item_example

# Rule to build all files generated by this target.
platforms/posix/CMakeFiles/px4.dir/build: bin/px4

.PHONY : platforms/posix/CMakeFiles/px4.dir/build

platforms/posix/CMakeFiles/px4.dir/clean:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix && $(CMAKE_COMMAND) -P CMakeFiles/px4.dir/cmake_clean.cmake
.PHONY : platforms/posix/CMakeFiles/px4.dir/clean

platforms/posix/CMakeFiles/px4.dir/depend:
	cd /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nics/Drone/PX4-Autopilot /home/nics/Drone/PX4-Autopilot/platforms/posix /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix /home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/CMakeFiles/px4.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : platforms/posix/CMakeFiles/px4.dir/depend

