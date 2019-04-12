#!/bin/bash
SESSION=bintel_sitl


# Split Panes
tmux -2 new-session -d -s $SESSION

tmux split-window -h  
tmux split-window -v  
tmux split-window -v 
tmux split-window -v 
tmux select-pane -t 0
tmux split-window -v  

tmux select-layout tiled

tmux send-keys -t 0 'cd ~/src/Firmware' C-m
tmux send-keys -t 0 'make px4_sitl_default gazebo' C-m
sleep 5
tmux send-keys -t 1 'roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"' C-m
tmux send-keys -t 2 "roslaunch bintel_ros cast_models.launch robot_urdf:='urdf/inteldrone_down.urdf'" C-m
tmux send-keys -t 3 "rosbag record /mavros/vision_pose/pose /mavros/imu/data /mavros/rc/out /mavros/rc/in /mavros/local_position/pose /mavros/local_position/velocity /tf"
tmux send-keys -t 3 "python test_arming_mavros.py"

tmux -2 attach-session -t $SESSION
