#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Simple script to set up a quick Mars Rover development environment

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/base_scripts/tools/base_common.sh

# Mapviz doesn't work on aarch64
if [ ! $(uname -m) == "aarch64" ]; then
	# Check if the mapproxy container is already running
	if [ $(docker ps | grep danielsnider/mapproxy | wc -l) -eq 0 ]; then
		# https://github.com/danielsnider/docker-mapproxy-googlemaps/tree/master
		printWarning "Starting the mapproxy container..."
		docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
	fi
fi

case $1 in
  	"down")
    	printWarning "Stopping the marsrover-ct container..."
    	docker compose -f docker/docker-compose.yaml down
    	;;
  	*)
		# Allow container to forward graphical displays to host
		xhost +

    	printInfo "Loading the marsrover-ct container..."
    	docker compose -f docker/docker-compose.yaml up -d

		# Check if a 'rover_dev' tmux session already exists
		if [ "$(docker exec -it marsrover-ct tmux list-sessions | grep rover_dev)" == "" ]; then

			# If not, create a new 'rover_dev' tmux session
			printWarning "Creating a new 'rover_dev' tmux session..."
			envsubst < $script_dir/base_scripts/tmuxp/rover_dev.yaml > $script_dir/base_scripts/tmuxp/tmp/rover_dev.yaml
			docker exec -it marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/rover_dev.yaml
		fi
		# Attach to the 'rover_dev' tmux session
		docker exec -it marsrover-ct tmux attach -t rover_dev
    ;;
esac
