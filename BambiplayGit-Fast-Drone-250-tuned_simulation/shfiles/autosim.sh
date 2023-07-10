if [ -z ${FastDrone250PATH} ]
then
	echo "FastDrone250PATH not set"
	exit
else
	echo "${FastDrone250PATH}/devel/setup.bash"
fi
roslaunch px4 mavros_posix_d435i_sitl.launch & sleep 5;
. "${FastDrone250PATH}/devel/setup.bash"
roslaunch px4ctrl run_ctrl_gazebo.launch & sleep 4;
roslaunch ego_planner rviz.launch & sleep 2;
roslaunch ego_planner single_run_in_gazebo.launch & sleep 10;
rosrun mavros mavcmd long 511 27 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd long 511 32 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd long 511 105 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd long 511 31 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd long 511 30 1000 0 0 0 0 0 & sleep 2;
rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1";

