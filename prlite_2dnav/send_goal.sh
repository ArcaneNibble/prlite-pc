# command line parameters are forwards goal, sideways goal, rotation goal
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped -1 -- '{header: {stamp: now, frame_id: base_link}, pose: {position: ['$1','$2',0.0], orientation: [0.0,0.0,'$3',1.0]}}'
