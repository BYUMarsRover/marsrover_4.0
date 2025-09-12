#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches the zed camera over SSH using the 'zed_launch' tmux session

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/tools/base_common.sh

# Check for an SSH connection to the rover
checkConnection # defined in base_common.sh

# Start the Docker container if not already running
if [ ! $(ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "docker ps" | grep -q "zed-ct") ]; then
    printWarning "Starting the zed-ct container on the rover..."
    # Check the architecture of the rover
    if [ $(ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "uname -m") == "aarch64" ]; then
        printInfo "Rover is running on ARM architecture, using the arm_orin image..."
        ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_2.0/zed_ws/docker/arm_orin && docker compose up -d"
    else
        printInfo "Rover is running on AMD architecture, using the amd_desktop image..."
        ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_2.0/zed_ws/docker/amd_desktop && docker compose up -d"
    fi
    ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_2.0/zed_ws/docker && docker compose up -d"
fi

# Send tmux commands to the rover over SSH
printInfo "Setting up the 'zed_launch' tmux session..."
# This envsubst allows for the use of environment variables in the tmuxp config
cd $script_dir && envsubst < tmuxp/autonomy/zed_launch.yaml > tmuxp/tmp/zed_launch.yaml
scp tmuxp/tmp/zed_launch.yaml $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover_2.0/base_scripts/tmuxp/tmp/
ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS \
    "docker exec zed-ct tmuxp load -d /home/marsrover-zed/.tmuxp/zed_launch.yaml"

# Attach to the 'zed_launch' tmux session (with mosh)
# https://github.com/mobile-shell/mosh
mosh $ROVER_USERNAME@$ROVER_IP_ADDRESS -- docker exec -it zed-ct tmux attach -t zed_launch

# Kill the tmux session on exit
ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "docker exec zed-ct tmux kill-session -t zed_launch"
