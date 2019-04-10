#!/bin/bash
SESSION=bintel_sitl

# Split Panes
tmux -2 new-session -d -s $SESSION

tmux split-window -h  # 2
tmux select-pane -t 0
tmux split-window -v  # 3
tmux select-pane -t 0 
tmux split-window -v # 4
tmux select-pane -t 0


tmux -2 attach-session -t $SESSION
