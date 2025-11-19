#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches missions on the base station using the 'base_launch' tmux session

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

# Start the Docker containers if not already running
if [ $(docker ps | grep marsrover-ct | wc -l) -eq 0 ]; then
	printWarning "Starting the marsrover-ct container..."
	cd ~/marsrover_4.0/docker && docker compose up -d
fi
if [ $(docker ps | grep mapproxy | wc -l) -eq 0 ]; then
    printWarning "Starting the mapproxy container..."
    docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
fi

# Allow container to forward graphical displays to host
xhost +

# Launch the specified mission configuration over SSH
case $mission in
    "autonomy")
        printInfo "Setting up the autonomy mission..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        cd $script_dir && envsubst < tmuxp/autonomy/base_launch.yaml > tmuxp/tmp/base_launch.yaml
        docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/base_launch.yaml
        ;;
    "servicing")
        printInfo "Setting up the servicing mission..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        cd $script_dir && envsubst < tmuxp/servicing/base_launch.yaml > tmuxp/tmp/base_launch.yaml
        docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/base_launch.yaml
        ;;
    "delivery")
        printInfo "Setting up the delivery mission..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        cd $script_dir && envsubst < tmuxp/delivery/base_launch.yaml > tmuxp/tmp/base_launch.yaml
        docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/base_launch.yaml
        ;;
    "science")
        printInfo "Setting up the science mission..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        cd $script_dir && envsubst < tmuxp/science/base_launch.yaml > tmuxp/tmp/base_launch.yaml
        docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/base_launch.yaml
        ;;
    *)
        printError "No mission specified"
        echo "Specify a mission using 'bash base_launch.sh -m <mission>' (ex. 'bash base_launch.sh -m autonomy')"
        exit 1
        ;;
esac

# Attach to the 'base_launch' tmux session
docker exec -it marsrover-ct tmux attach -t base_launch

# Kill the tmux session on exit
docker exec marsrover-ct tmux kill-session -t base_launch
