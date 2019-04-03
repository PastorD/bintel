#!/bin/bash
SESSION="runBintel"

############################## PANES #########################################

# Split Panes
tmux -2 new-session -d -s $SESSION
tmux split-window -v -p 66  
tmux select-pane -t 1
tmux split-window -v  -p 50
tmux select-pane -t 0
tmux split-window -h -p 50
tmux select-pane -t 2
tmux split-window -h -p 50
tmux select-pane -t 4
tmux split-window -h -p 50


#######################################################################################
############################# Drone  ##################################################
#######################################################################################
tmux send-keys -t 0 "sudo ntpq -p" C-m
export DRONE=bintel
tmux send-keys -t 0 "./increase_rates.sh" C-m
sleep 2


############################# Mavros ##############################################
tmux send-keys -t 1 "roslaunch bintel_ros mavros_optitrack.launch" C-m

sleep 2

############################# Optitrack ###########################################
tmux send-keys -t 0 "roslaunch bintel_ros vrpn_optitrack_positioning.launch" C-m

sleep 2



#######################################################################################
############################# Ground Station ##########################################
#######################################################################################
tmux send-keys -t 0 "sudo ntpq -p" C-m
export DRONE=bintel


############################# QGroundControl ######################################
# use this file to open Qgroundcontrol
tmux send-keys -t 0 "~/Downloads/QGroundControl.AppImage" C-m



############################# QGroundControl ######################################

# use this file to show the cast environment
tmux send-keys -t 0 "roslaunch bintel_ros cast_models.launch robot_urdf:='urdf/inteldrone_down.urdf'" C-m




########################### Terminate #############################################

tmux select-pane -t 9

tmux -2 attach-session -t $SESSION
