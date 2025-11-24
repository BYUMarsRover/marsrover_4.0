#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Reset the USB devices on the rover over SSH

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/base_common.sh

# Check for an SSH connection to the rover
checkConnection # defined in base_common.sh

ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_4.0/docker/udev && bash usb_reset.sh"
