roslaunch px4ctrl drone2_run_ctrl_gazebo.launch & sleep 4;
roslaunch ego_planner drone2_run_in_gazebo.launch & sleep 4;
rosrun mavros mavcmd -n drone2/mavros long 511 27 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd -n drone2/mavros long 511 32 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd -n drone2/mavros long 511 105 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd -n drone2/mavros long 511 31 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd -n drone2/mavros long 511 30 1000 0 0 0 0 0 & sleep 2;
rostopic pub -1  /drone2/px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1";