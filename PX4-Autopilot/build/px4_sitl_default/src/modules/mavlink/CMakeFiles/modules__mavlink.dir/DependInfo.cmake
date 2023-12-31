# The set of languages for which implicit dependencies are needed:
set(CMAKE_DEPENDS_LANGUAGES
  "C"
  "CXX"
  )
# The set of files for implicit dependencies of each language:
set(CMAKE_DEPENDS_CHECK_C
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink.c" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink.c.o"
  )
set(CMAKE_C_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_C
  "CONFIG_ARCH_BOARD_PX4_SITL"
  "ENABLE_LOCKSTEP_SCHEDULER"
  "MODULE_NAME=\"mavlink\""
  "PX4_BOARD_NAME=\"PX4_SITL\""
  "PX4_MAIN=mavlink_app_main"
  "__CUSTOM_FILE_IO__"
  "__PX4_LINUX"
  "__PX4_POSIX"
  "__STDC_FORMAT_MACROS"
  "noreturn_function=__attribute__((noreturn))"
  )

# The include file search paths:
set(CMAKE_C_TARGET_INCLUDE_PATH
  "../../boards/px4/sitl/src"
  "../../platforms/posix/src/px4/common/include"
  "."
  "src/lib"
  "../../platforms/posix/src/px4/generic/generic/include"
  "../../platforms/common/include"
  "../../src"
  "../../src/include"
  "../../src/lib"
  "../../src/lib/matrix"
  "../../src/modules"
  "../../platforms/posix/include"
  "external/Install/include"
  "../../mavlink/include/mavlink"
  "../../src/lib/ecl"
  )
set(CMAKE_DEPENDS_CHECK_CXX
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_command_sender.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_command_sender.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_ftp.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_ftp.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_log_handler.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_log_handler.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_main.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_main.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_messages.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_messages.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_mission.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_mission.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_parameters.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_parameters.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_rate_limiter.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_rate_limiter.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_receiver.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_receiver.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_shell.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_shell.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_simple_analyzer.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_simple_analyzer.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_stream.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_stream.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_timesync.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_timesync.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/mavlink_ulog.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/mavlink_ulog.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/mavlink/tune_publisher.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/mavlink/CMakeFiles/modules__mavlink.dir/tune_publisher.cpp.o"
  )
set(CMAKE_CXX_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_CXX
  "CONFIG_ARCH_BOARD_PX4_SITL"
  "ENABLE_LOCKSTEP_SCHEDULER"
  "MODULE_NAME=\"mavlink\""
  "PX4_BOARD_NAME=\"PX4_SITL\""
  "PX4_MAIN=mavlink_app_main"
  "__CUSTOM_FILE_IO__"
  "__PX4_LINUX"
  "__PX4_POSIX"
  "__STDC_FORMAT_MACROS"
  "noreturn_function=__attribute__((noreturn))"
  )

# The include file search paths:
set(CMAKE_CXX_TARGET_INCLUDE_PATH
  "../../boards/px4/sitl/src"
  "../../platforms/posix/src/px4/common/include"
  "."
  "src/lib"
  "../../platforms/posix/src/px4/generic/generic/include"
  "../../platforms/common/include"
  "../../src"
  "../../src/include"
  "../../src/lib"
  "../../src/lib/matrix"
  "../../src/modules"
  "../../platforms/posix/include"
  "external/Install/include"
  "../../mavlink/include/mavlink"
  "../../src/lib/ecl"
  )

# Targets to which this target links.
set(CMAKE_TARGET_LINKED_INFO_FILES
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/src/px4/common/CMakeFiles/px4_layer.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/common/CMakeFiles/px4_platform.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/systemlib/CMakeFiles/systemlib.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/airspeed/CMakeFiles/airspeed.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/drivers/accelerometer/CMakeFiles/drivers_accelerometer.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/drivers/barometer/CMakeFiles/drivers_barometer.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/drivers/gyroscope/CMakeFiles/drivers_gyroscope.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/drivers/magnetometer/CMakeFiles/drivers_magnetometer.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/conversion/CMakeFiles/conversion.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/ecl/geo/CMakeFiles/ecl_geo.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/version/CMakeFiles/version.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/drivers/device/CMakeFiles/drivers__device.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/common/px4_work_queue/CMakeFiles/px4_work_queue.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/src/px4/common/px4_daemon/CMakeFiles/px4_daemon.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/src/px4/common/lockstep_scheduler/CMakeFiles/lockstep_scheduler.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB/CMakeFiles/modules__uORB.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/cdev/CMakeFiles/cdev.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/common/work_queue/CMakeFiles/work_queue.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/parameters/CMakeFiles/parameters.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/perf/CMakeFiles/perf.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/parameters/tinybson/CMakeFiles/tinybson.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/msg/CMakeFiles/uorb_msgs.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/boards/px4/sitl/src/CMakeFiles/drivers_board.dir/DependInfo.cmake"
  )

# Fortran module output directory.
set(CMAKE_Fortran_TARGET_MODULE_DIR "")
