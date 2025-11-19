#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Pulls the the latest Docker image onto the rover

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/base_common.sh

# Check for an SSH connection to the rover
checkConnection # defined in base_common.sh

rover_arch=$(ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "uname -m")
if [[ "$rover_arch" == "aarch64" ]]; then
    rover_id="linux/arm64"
else
    rover_id="linux/amd64"
fi

printInfo "Pulling the $rover_id version of the Docker image onto the base station..."
docker pull --platform $rover_id byuawesomerover/marsrover_4.0:latest
docker image list

printInfo "Saving the $rover_id Docker image to a zip file for transfer (this takes a while)..."
docker save byuawesomerover/marsrover_4.0:latest | gzip > marsrover_4.0.tar.gz

printInfo "Sending the Docker image to the rover..."
scp marsrover_4.0.tar.gz $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover_4.0/docker
rm marsrover_4.0.tar.gz

printInfo "Unzipping the Docker image on the rover..."
ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_4.0/docker && gunzip marsrover_4.0.tar.gz"

printInfo "Loading the Docker image on the rover..."
ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_4.0/docker && docker load < marsrover_4.0.tar"
ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_4.0/docker && rm marsrover_4.0.tar"
ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "yes | docker image prune"
ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "docker image list"

printInfo "Pulling the linux/amd64 Docker image onto the base station..."
docker pull --platform linux/amd64 byuawesomerover/marsrover_4.0:latest
yes | docker image prune
docker image list
