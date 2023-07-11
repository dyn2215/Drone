# The set of languages for which implicit dependencies are needed:
set(CMAKE_DEPENDS_LANGUAGES
  "CXX"
  )
# The set of files for implicit dependencies of each language:
set(CMAKE_DEPENDS_CHECK_CXX
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/PreFlightCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/PreFlightCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/accelerometerCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/accelerometerCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/airframeCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/airframeCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/airspeedCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/airspeedCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/baroCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/baroCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/cpuResourceCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/cpuResourceCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/ekf2Check.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/ekf2Check.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/failureDetectorCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/failureDetectorCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/gyroCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/gyroCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/imuConsistencyCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/imuConsistencyCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/magConsistencyCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/magConsistencyCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/magnetometerCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/magnetometerCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/manualControlCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/manualControlCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/powerCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/powerCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/preArmCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/preArmCheck.cpp.o"
  "/home/nics/Drone/PX4-Autopilot/src/modules/commander/Arming/PreFlightCheck/checks/rcCalibrationCheck.cpp" "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/PreFlightCheck/CMakeFiles/PreFlightCheck.dir/checks/rcCalibrationCheck.cpp.o"
  )
set(CMAKE_CXX_COMPILER_ID "GNU")

# Preprocessor definitions for this target.
set(CMAKE_TARGET_DEFINITIONS_CXX
  "CONFIG_ARCH_BOARD_PX4_SITL"
  "ENABLE_LOCKSTEP_SCHEDULER"
  "MODULE_NAME=\"PreFlightCheck\""
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
  "../../src/modules/commander/Arming/PreFlightCheck"
  "../../src/modules/commander/Arming/ArmAuthorization"
  "../../src/modules/commander/Arming/HealthFlags"
  )

# Targets to which this target links.
set(CMAKE_TARGET_LINKED_INFO_FILES
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/common/CMakeFiles/px4_platform.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/msg/CMakeFiles/uorb_msgs.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/ArmAuthorization/CMakeFiles/ArmAuthorization.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/commander/Arming/HealthFlags/CMakeFiles/HealthFlags.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/sensor_calibration/CMakeFiles/sensor_calibration.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/conversion/CMakeFiles/conversion.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/modules/uORB/CMakeFiles/modules__uORB.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/src/px4/common/CMakeFiles/px4_layer.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/systemlib/CMakeFiles/systemlib.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/cdev/CMakeFiles/cdev.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/common/px4_work_queue/CMakeFiles/px4_work_queue.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/src/px4/common/px4_daemon/CMakeFiles/px4_daemon.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/posix/src/px4/common/lockstep_scheduler/CMakeFiles/lockstep_scheduler.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/platforms/common/work_queue/CMakeFiles/work_queue.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/boards/px4/sitl/src/CMakeFiles/drivers_board.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/parameters/CMakeFiles/parameters.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/perf/CMakeFiles/perf.dir/DependInfo.cmake"
  "/home/nics/Drone/PX4-Autopilot/build/px4_sitl_default/src/lib/parameters/tinybson/CMakeFiles/tinybson.dir/DependInfo.cmake"
  )

# Fortran module output directory.
set(CMAKE_Fortran_TARGET_MODULE_DIR "")
