/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#pragma once

#include <stddef.h>

#include <uORB/uORB.h>

static constexpr size_t ORB_TOPICS_COUNT{180};
static constexpr size_t orb_topics_count() { return ORB_TOPICS_COUNT; }

/*
 * Returns array of topics metadata
 */
extern const struct orb_metadata *const *orb_get_topics() __EXPORT;

enum class ORB_ID : uint8_t {
	actuator_armed = 0,
	actuator_controls = 1,
	actuator_controls_0 = 2,
	actuator_controls_1 = 3,
	actuator_controls_2 = 4,
	actuator_controls_3 = 5,
	actuator_controls_4 = 6,
	actuator_controls_5 = 7,
	actuator_controls_virtual_fw = 8,
	actuator_controls_virtual_mc = 9,
	actuator_outputs = 10,
	adc_report = 11,
	airspeed = 12,
	airspeed_validated = 13,
	battery_status = 14,
	camera_capture = 15,
	camera_trigger = 16,
	camera_trigger_secondary = 17,
	cellular_status = 18,
	collision_constraints = 19,
	collision_report = 20,
	commander_state = 21,
	control_allocator_status = 22,
	cpuload = 23,
	debug_array = 24,
	debug_key_value = 25,
	debug_value = 26,
	debug_vect = 27,
	differential_pressure = 28,
	distance_sensor = 29,
	ekf2_timestamps = 30,
	ekf_gps_drift = 31,
	esc_report = 32,
	esc_status = 33,
	estimator_attitude = 34,
	estimator_global_position = 35,
	estimator_innovation_test_ratios = 36,
	estimator_innovation_variances = 37,
	estimator_innovations = 38,
	estimator_local_position = 39,
	estimator_odometry = 40,
	estimator_optical_flow_vel = 41,
	estimator_selector_status = 42,
	estimator_sensor_bias = 43,
	estimator_states = 44,
	estimator_status = 45,
	estimator_status_flags = 46,
	estimator_visual_odometry_aligned = 47,
	follow_target = 48,
	fw_virtual_attitude_setpoint = 49,
	generator_status = 50,
	geofence_result = 51,
	gps_dump = 52,
	gps_inject_data = 53,
	home_position = 54,
	hover_thrust_estimate = 55,
	input_rc = 56,
	iridiumsbd_status = 57,
	irlock_report = 58,
	landing_gear = 59,
	landing_target_innovations = 60,
	landing_target_pose = 61,
	led_control = 62,
	log_message = 63,
	logger_status = 64,
	mag_worker_data = 65,
	manual_control_setpoint = 66,
	manual_control_switches = 67,
	mavlink_log = 68,
	mc_virtual_attitude_setpoint = 69,
	mission = 70,
	mission_result = 71,
	mount_orientation = 72,
	multirotor_motor_limits = 73,
	navigator_mission_item = 74,
	obstacle_distance = 75,
	obstacle_distance_fused = 76,
	offboard_control_mode = 77,
	onboard_computer_status = 78,
	optical_flow = 79,
	orb_multitest = 80,
	orb_test = 81,
	orb_test_large = 82,
	orb_test_medium = 83,
	orb_test_medium_multi = 84,
	orb_test_medium_queue = 85,
	orb_test_medium_queue_poll = 86,
	orb_test_medium_wrap_around = 87,
	orbit_status = 88,
	parameter_update = 89,
	ping = 90,
	position_controller_landing_status = 91,
	position_controller_status = 92,
	position_setpoint = 93,
	position_setpoint_triplet = 94,
	power_button_state = 95,
	power_monitor = 96,
	pwm_input = 97,
	px4io_status = 98,
	qshell_req = 99,
	qshell_retval = 100,
	radio_status = 101,
	rate_ctrl_status = 102,
	rc_channels = 103,
	rc_parameter_map = 104,
	rpm = 105,
	rtl_flight_time = 106,
	safety = 107,
	satellite_info = 108,
	sensor_accel = 109,
	sensor_accel_fifo = 110,
	sensor_baro = 111,
	sensor_combined = 112,
	sensor_correction = 113,
	sensor_gps = 114,
	sensor_gyro = 115,
	sensor_gyro_fft = 116,
	sensor_gyro_fifo = 117,
	sensor_mag = 118,
	sensor_preflight_mag = 119,
	sensor_selection = 120,
	sensors_status_imu = 121,
	system_power = 122,
	takeoff_status = 123,
	task_stack_info = 124,
	tecs_status = 125,
	telemetry_status = 126,
	test_motor = 127,
	timesync = 128,
	timesync_status = 129,
	trajectory_bezier = 130,
	trajectory_setpoint = 131,
	trajectory_waypoint = 132,
	transponder_report = 133,
	tune_control = 134,
	uavcan_parameter_request = 135,
	uavcan_parameter_value = 136,
	ulog_stream = 137,
	ulog_stream_ack = 138,
	vehicle_acceleration = 139,
	vehicle_actuator_setpoint = 140,
	vehicle_air_data = 141,
	vehicle_angular_acceleration = 142,
	vehicle_angular_acceleration_setpoint = 143,
	vehicle_angular_velocity = 144,
	vehicle_angular_velocity_groundtruth = 145,
	vehicle_attitude = 146,
	vehicle_attitude_groundtruth = 147,
	vehicle_attitude_setpoint = 148,
	vehicle_command = 149,
	vehicle_command_ack = 150,
	vehicle_constraints = 151,
	vehicle_control_mode = 152,
	vehicle_global_position = 153,
	vehicle_global_position_groundtruth = 154,
	vehicle_gps_position = 155,
	vehicle_imu = 156,
	vehicle_imu_status = 157,
	vehicle_land_detected = 158,
	vehicle_local_position = 159,
	vehicle_local_position_groundtruth = 160,
	vehicle_local_position_setpoint = 161,
	vehicle_magnetometer = 162,
	vehicle_mocap_odometry = 163,
	vehicle_odometry = 164,
	vehicle_rates_setpoint = 165,
	vehicle_roi = 166,
	vehicle_status = 167,
	vehicle_status_flags = 168,
	vehicle_thrust_setpoint = 169,
	vehicle_torque_setpoint = 170,
	vehicle_trajectory_bezier = 171,
	vehicle_trajectory_waypoint = 172,
	vehicle_trajectory_waypoint_desired = 173,
	vehicle_vision_attitude = 174,
	vehicle_visual_odometry = 175,
	vtol_vehicle_status = 176,
	wheel_encoders = 177,
	wind_estimate = 178,
	yaw_estimator_status = 179,

	INVALID
};

const struct orb_metadata *get_orb_meta(ORB_ID id);
