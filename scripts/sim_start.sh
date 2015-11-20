#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongodb'
tmux new-window -t $SESSION:2 -n 'simulation'
tmux new-window -t $SESSION:3 -n '2d_nav'
tmux new-window -t $SESSION:4 -n 'cameras'
tmux new-window -t $SESSION:5 -n 'strands_ui'
tmux new-window -t $SESSION:6 -n 'backtrack_behaviour'
tmux new-window -t $SESSION:7 -n 'topo_nav'
tmux new-window -t $SESSION:8 -n 'fremen'
tmux new-window -t $SESSION:9 -n 'inject_pose'
tmux new-window -t $SESSION:10 -n 'scene_generator'
tmux new-window -t $SESSION:11 -n 'planner'
tmux new-window -t $SESSION:12 -n 'executioner'
tmux new-window -t $SESSION:13 -n 'scheduler'
tmux new-window -t $SESSION:14 -n 'scheduler_creator'
tmux new-window -t $SESSION:15 -n 'services'


tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch mongodb_store mongodb_store.launch db_path:=$HOME/mongodb_store"

tmux select-window -t $SESSION:2
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch strands_morse uol_ww_morse.launch"

tmux select-window -t $SESSION:3
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch strands_morse uol_ww_nav2d.launch"

tmux select-window -t $SESSION:4
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch strands_morse generate_camera_topics.launch"

tmux select-window -t $SESSION:5
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch strands_ui strands_ui.launch"

tmux select-window -t $SESSION:6
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch backtrack_behaviour backtrack.launch"

tmux select-window -t $SESSION:7
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch topological_navigation topological_navigation_empty_map.launch map:=empty_map mon_nav_config_file:=$(rospack find strands_recovery_behaviours)/config/monitored_nav_config.yaml"

tmux select-window -t $SESSION:8
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 rosrun spatiotemporalexploration fremengrid"

tmux select-window -t $SESSION:9
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 rosrun spatiotemporalexploration inject_pose_server.py;"

tmux select-window -t $SESSION:10
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch spatiotemporalexploration simulation.launch"

tmux select-window -t $SESSION:11
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch spatiotemporalexploration planner.launch"

tmux select-window -t $SESSION:12
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 roslaunch spatiotemporalexploration executioner.launch"

tmux select-window -t $SESSION:13
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 rosrun spatiotemporalexploration exploration_scheduler"

tmux select-window -t $SESSION:14
tmux send-keys "source ~/catkin_ws/devel/setup.bash; DISPLAY=:0 rosrun spatiotemporalexploration exploration_schedule_creator"

tmux select-window -t $SESSION:15
tmux send-keys "source ~/catkin_ws/devel/setup.bash;clear;"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse off
