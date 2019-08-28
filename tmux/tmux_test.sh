#!/bin/bash
SESSION="runBintel_simulation"

############################## PANES #########################################

# Split Panes
tmux -2 new-session -d -s $SESSION




############################# Optitrack ###########################################
tmux send-keys -t 0 "roslaunch bintel_ros vrpn_optitrack_positioning.launch" C-m

############################# QGroundControl ######################################
tmux split-window -h
tmux select-pane -t 0
# use this file to open Qgroundcontrol
tmux send-keys -t 0 "~/Downloads/QGroundControl.AppImage" C-m



############################# QGroundControl ######################################
# use this file to show the cast environment
tmux split-window -h
tmux send-keys -t 0  "roslaunch bintel_ros cast_models.launch robot_urdf:='urdf/inteldrone_down.urdf'" C-m


tmux select-pane -t 0
tmux -2 attach-session -t $SESSION
