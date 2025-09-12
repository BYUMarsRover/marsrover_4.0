#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Calibrate the ZED camera magnetometer on the rover

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/base_common.sh

# Check for an SSH connection to the rover
checkConnection # defined in base_common.sh

# Allow container to forward graphical displays to host
xhost +

# Launch the ZED Sensor Viewer over SSH
ssh -t -X $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd /usr/local/zed/tools && ./ZED_Sensor_Viewer"
