#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches missions on the rover over SSH using the 'rover_launch' tmux session

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/tools/base_common.sh

# Check for a "-m <mission>" argument
while getopts ":m:" opt; do
    case $opt in
        m)
            mission=$OPTARG
        ;;
    esac
done

# Check for an SSH connection to the rover
checkConnection # defined in base_common.sh

# Start the Docker container if not already running
if [ ! $(ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "docker ps" | grep -q "marsrover-ct") ]; then
    printWarning "Starting the marsrover-ct container on the rover..."
    ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_2.0/docker && docker compose up -d"
fi

# Launch the specified mission configuration over SSH
case $mission in
    "autonomy")
        printInfo "Setting up the autonomy mission..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        cd $script_dir && envsubst < tmuxp/autonomy/rover_launch.yaml > tmuxp/tmp/rover_launch.yaml
        scp tmuxp/tmp/rover_launch.yaml $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover_2.0/base_scripts/tmuxp/tmp/
        ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS \
            "docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/rover_launch.yaml"
        ;;
    "servicing")
        printInfo "Setting up the servicing mission..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        cd $script_dir && envsubst < tmuxp/servicing/rover_launch.yaml > tmuxp/tmp/rover_launch.yaml
        scp tmuxp/tmp/rover_launch.yaml $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover_2.0/base_scripts/tmuxp/tmp/
        ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS \
            "docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/rover_launch.yaml"
        ;;
    "delivery")
        printInfo "Setting up the delivery mission..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        cd $script_dir && envsubst < tmuxp/delivery/rover_launch.yaml > tmuxp/tmp/rover_launch.yaml
        scp tmuxp/tmp/rover_launch.yaml $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover_2.0/base_scripts/tmuxp/tmp/
        ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS \
            "docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/rover_launch.yaml"
        ;;
    "science")
        printInfo "Setting up the science mission..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        cd $script_dir && envsubst < tmuxp/science/rover_launch.yaml > tmuxp/tmp/rover_launch.yaml
        scp tmuxp/tmp/rover_launch.yaml $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover_2.0/base_scripts/tmuxp/tmp/
        ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS \
            "docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/rover_launch.yaml"
        ;;
    *)
        printError "No mission specified"
        echo "Specify a mission using 'bash rover_launch.sh -m <mission>' (ex. 'bash rover_launch.sh -m autonomy')"
        exit 1
        ;;
esac

# Attach to the 'rover_launch' tmux session (with mosh)
# https://github.com/mobile-shell/mosh
mosh $ROVER_USERNAME@$ROVER_IP_ADDRESS -- docker exec -it marsrover-ct tmux attach -t rover_launch

# Kill the tmux session on exit
ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "docker exec marsrover-ct tmux kill-session -t rover_launch"
