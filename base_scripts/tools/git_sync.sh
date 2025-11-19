#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Pulls the base station's local git branch onto the rover

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/base_common.sh

# Check for an SSH connection to the rover
checkConnection # defined in base_common.sh

if ! ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_4.0 && git remote -v" | grep -q "base"
then
    printError "No upstream git remote 'base' found on the rover"
    echo "Please run 'bash setup_git.sh' to set up the upstream git remote"

    exit 1
fi

export current_branch=$(git branch --show-current)

ssh -t marsrover@$ROVER_IP_ADDRESS "cd ~/marsrover_4.0 && git checkout $current_branch"
ssh -t marsrover@$ROVER_IP_ADDRESS "cd ~/marsrover_4.0 && git pull base ${current_branch}"

