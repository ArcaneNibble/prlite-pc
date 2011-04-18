# command line parameters are vx, vy, vtheta
rostopic pub /cmd_vel geometry_msgs/Twist -1 -- '{linear: ['$1', '$2', 0], angular: [0, 0, '$3']}'
