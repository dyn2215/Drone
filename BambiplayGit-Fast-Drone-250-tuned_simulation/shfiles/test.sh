source ~/.bashrc & sleep 1;

# decide the mean and stddev for the Gaussian distribution
# x,y stddev in meter
stddevx=0.1
stddevy=0.1

# yaw, in degree
meanY=0.0
stddevYdeg=5.0
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