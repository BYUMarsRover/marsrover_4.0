#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Set up the base station as an upstream git remote for the rover

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/base_common.sh

# Check for an SSH connection to the rover
checkConnection # defined in base_common.sh

ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_4.0 && \
    git remote add base marsrover@192.168.1.111:marsrover_4.0"
