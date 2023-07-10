roslaunch px4 mavros_503_single.launch & sleep 10;
roslaunch px4 turtlebot3.launch & sleep 2;
roslaunch px4ctrl run_ctrl_gazebo.launch & sleep 4;
roslaunch ego_planner rviz.launch & sleep 2;
roslaunch ego_planner single_run_in_gazebo.launch & sleep 10;
rosrun mavros mavcmd long 511 27 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd long 511 32 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd long 511 105 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd long 511 31 1000 0 0 0 0 0 & sleep 2;
rosrun mavros mavcmd long 511 30 1000 0 0 0 0 0 & sleep 2;
rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1";

