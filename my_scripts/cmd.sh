#!/bin/bash

# Name of the tmux session
SESSION_NAME="test_session"

# Create a new tmux session
tmux new-session -d -s $SESSION_NAME

# Split the window into multiple panes
tmux split-window -h
tmux split-window -v
tmux split-window -v

# Select the first pane
tmux select-pane -t 0

# Send commands to the panes
tmux send-keys -t 0 'echo "This is pane 0"' C-m
tmux send-keys -t 1 'echo "This is pane 1"' C-m
tmux send-keys -t 2 'echo "This is pane 2"' C-m
tmux send-keys -t 3 'echo "This is pane 3"' C-m

# Attach to the session
#tmux attach -t $SESSION_NAME
