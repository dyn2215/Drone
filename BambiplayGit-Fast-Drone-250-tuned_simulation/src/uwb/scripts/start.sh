source ~/uwb_catkin_ws/devel/setup.bash
roslaunch uwb start_in_exp.launch self_id:=$DRONE_ID init_x:=$FORM_X init_y:=$FORM_Y init_z:=$FORM_Z
