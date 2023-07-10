#send goal for 3 q250 drones
gnome-terminal -- bash -c  "rostopic pub -1  /drone_0_ego_planner_node/move_base_simple/goal geometry_msgs/PoseStamped \ '{header: {frame_id: "world"},pose: {position:{x: 0,y: 0,z: 0},orientation: {x: 0,y: 0,z: 0,w: 1}}}';
;exec bash;" &

gnome-terminal -- bash -c  "rostopic pub -1  /drone_1_ego_planner_node/move_base_simple/goal geometry_msgs/PoseStamped \ '{header: {frame_id: "world"},pose: {position:{x: 0,y: 1,z: 0},orientation: {x: 0,y: 0,z: 0,w: 1}}}';
;exec bash;" &

gnome-terminal -- bash -c  "rostopic pub -1  /drone_2_ego_planner_node/move_base_simple/goal geometry_msgs/PoseStamped \ '{header: {frame_id: "world"},pose: {position:{x: 0,y: 2,z: 0},orientation: {x: 0,y: 0,z: 0,w: 1}}}';
;exec bash;" 