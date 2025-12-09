#!/bin/bash

# This script should be run on the rover to install a systemd service monitoring pin 7 and triggering soft shutdown.

echo "#!/bin/bash

# This script monitors pin 7 and triggers a soft shutdown when the pin is activated.

SOFT_SHUTDOWN_PIN=7 # Pin 7 is pull-down by default
PIN_PATH="/sys/class/gpio/gpio$SOFT_SHUTDOWN_PIN"

# Export the pin if not already exported
if [ ! -d "$PIN_PATH" ]; then
    echo $SOFT_SHUTDOWN_PIN > /sys/class/gpio/export 2>/dev/null || {
        echo "Failed to export GPIO pin $SOFT_SHUTDOWN_PIN"
        exit 1
    }
    sleep 0.5  # Give the system time to create the GPIO directory
fi

# Set pin direction to input
echo "in" > $PIN_PATH/direction

while true; do
    # Check if the pin path still exists before reading
    if [ ! -f "$PIN_PATH/value" ]; then
        echo "GPIO pin path is no longer available"
        exit 1
    fi
    
    if [ "$(cat $PIN_PATH/value)" -eq 1 ]; then
        echo "Soft shutdown triggered"
        shutdown -h now
        break
    fi
    sleep 1
done" > /usr/local/bin/soft_shutdown.sh
chmod +x /usr/local/bin/soft_shutdown.sh

echo "[Unit]
Description=Soft Shutdown Service
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/local/bin/soft_shutdown.sh
User=root
Restart=always
RestartSec=10" > /etc/systemd/system/soft_shutdown.service

systemctl enable soft_shutdown.service
systemctl start soft_shutdown.service