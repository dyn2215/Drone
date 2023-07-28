source ~/.bashrc & sleep 1;

# decide the mean and stddev for the Gaussian distribution
# x,y stddev in meter
stddevx=0.1
stddevy=0.1

# yaw, in degree
meanY=0.0
stddevYdeg=10.0
stddevY=$stddevYdeg*3.1415926/180.0

# Generate a random number from a Gaussian distribution with mean and stddev
drone0_init_x=$(python -c "import random; print(random.gauss(0, $stddevx))")
drone0_init_y=$(python -c "import random; print(random.gauss(0, $stddevy))")
drone0_init_Y=$(python -c "import random; print(random.gauss($meanY, $stddevY))")

drone1_init_x=$(python -c "import random; print(random.gauss(0, $stddevx))")
drone1_init_y=$(python -c "import random; print(random.gauss(1, $stddevy))")
drone1_init_Y=$(python -c "import random; print(random.gauss($meanY, $stddevY))")

drone2_init_x=$(python -c "import random; print(random.gauss(0, $stddevx))")
drone2_init_y=$(python -c "import random; print(random.gauss(2, $stddevy))")
drone2_init_Y=$(python -c "import random; print(random.gauss($meanY, $stddevY))")

# I want to save them in a single file, please show me how to do it
echo -e "hyperparameters:\n stddevx=$stddevx\n stddevy=$stddevy\n stddevYdeg=$stddevYdeg" > init_pos.txt
echo -e "drone0 position:\n drone0_init_x=$drone0_init_x\n drone0_init_y=$drone0_init_y\n drone0_init_Y=$drone0_init_Y" >> init_pos.txt
echo -e "drone1 position:\n drone1_init_x=$drone1_init_x\n drone1_init_y=$drone1_init_y\n drone1_init_Y=$drone1_init_Y" >> init_pos.txt
echo -e "drone2 position:\n drone2_init_x=$drone2_init_x\n drone2_init_y=$drone2_init_y\n drone2_init_Y=$drone2_init_Y" >> init_pos.txt

# launch the simulation with the generated random number
roslaunch px4 multi_q250_mavros_sitl.launch \
drone0_init_x:=$drone0_init_x \
drone0_init_y:=$drone0_init_y \
drone0_init_Y:=$drone0_init_Y \
drone1_init_x:=$drone1_init_x \
drone1_init_y:=$drone1_init_y \
drone1_init_Y:=$drone1_init_Y \
drone2_init_x:=$drone2_init_x \
drone2_init_y:=$drone2_init_y \
drone2_init_Y:=$drone2_init_Y & sleep 10;

# rosparam set /uav_init_pos [0,0,0,1,0,2];
# rosparam set /uav_init_pos [{$drone0_init_x}, {$drone0_init_y}, {$drone1_init_x}, {$drone1_init_y}, {$drone2_init_x}, {$drone2_init_y}, {$drone0_init_Y}, {$drone1_init_Y}, {$drone2_init_Y}];
# rosparam set /uav_init_pos [$drone0_init_x, $drone0_init_y, $drone1_init_x, $drone1_init_y, $drone2_init_x, $drone2_init_y]
paramcmdstr=$(printf "rosparam set /uav_init_pos [%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]" $drone0_init_x $drone0_init_y $drone1_init_x $drone1_init_y $drone2_init_x $drone2_init_y $drone0_init_Y $drone1_init_Y $drone2_init_Y)
eval $paramcmdstr

roslaunch ego_planner swarm_odom_publisher.launch & sleep 5;
# roslaunch drone_detect drone_detect_3drones.launch & sleep 2;
roslaunch rosmsg_tcp_bridge fake_swarm_bridge.launch & sleep 2;

# . load_3drone_multiprocess.sh & sleep 15;
./drone0.sh & sleep 5;
./drone1.sh & sleep 5;
./drone2.sh & sleep 5;

# start uwb simulation, drift simulation and drift correction
roslaunch uwb uwb_launch_all.launch & sleep 3;

roslaunch ego_planner rviz.launch & sleep 2;

# sleep 20;
# rostopic pub -1 /drone_0_ego_planner_node/move_base_simple/goal geometry_msgs/PoseStamped ‘{header:{stamp:now, frame_id:"world"}, pose:{position:{x:25.0, y:2.0, z:0.0}, orientation:{x:0.0, y:0.0, z:0.0, w:1.0}}}’
# rostopic pub -1 /drone_1_ego_planner_node/move_base_simple/goal geometry_msgs/PoseStamped ‘{header:{stamp:now, frame_id:"world"}, pose:{position:{x:25.0, y:1.0, z:0.0}, orientation:{x:0.0, y:0.0, z:0.0, w:1.0}}}’
# rostopic pub -1 /drone_2_ego_planner_node/move_base_simple/goal geometry_msgs/PoseStamped ‘{header:{stamp:now, frame_id:"world"}, pose:{position:{x:25.0, y:0.0, z:0.0}, orientation:{x:0.0, y:0.0, z:0.0, w:1.0}}}’
# rostopic pub --once /start std_msgs/String "go"