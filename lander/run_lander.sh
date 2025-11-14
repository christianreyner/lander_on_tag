#!/usr/bin/env bash
set -euo pipefail

SESSION="lander_stack"
PY_ENV="${HOME}/ardupilot/opt/bin/activate"

# Start fresh session
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi

tmux new-session -d -s "$SESSION"

# Enable mouse in this session (click to focus/resize)
tmux set-option -t "$SESSION" mouse on

# Pane 1: MAVProxy (interactive)
tmux send-keys -t "$SESSION":0.0 "source '$PY_ENV' && mavproxy.py --master=/dev/ttyACM1 --master=/dev/ttyACM0 --out=udp:100.66.209.91:14550 --out=udp:0.0.0.0:14550" C-m

# Pane 2: tracker
tmux split-window -h -t "$SESSION":0
tmux send-keys -t "$SESSION":0.1 "sudo ./build/lander --viz" C-m

# Pane 3: lt_bridge
tmux split-window -v -t "$SESSION":0.1
tmux send-keys -t "$SESSION":0.2 "source '$PY_ENV' && python3 lt_bridge.py" C-m

tmux select-layout -t "$SESSION":0 tiled

# Focus MAVProxy pane by default and attach
tmux select-pane -t "$SESSION":0.0
tmux attach -t "$SESSION"
