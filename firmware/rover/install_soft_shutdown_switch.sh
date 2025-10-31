#!/bin/bash

# This script should be run on the rover to install a systemd service monitoring pin 40 and triggering soft shutdown. If gpiod is not already installed, this script requires internet access.

sudo apt install -y gpiod

echo "#!/bin/bash

# This script monitors pin 40 and triggers a soft shutdown when the pin is activated.

SOFT_SHUTDOWN_PIN=40
PIN_PATH="/sys/class/gpio/gpio$SOFT_SHUTDOWN_PIN"

echo $SOFT_SHUTDOWN_PIN > /sys/class/gpio/export
echo "in" > $PIN_PATH/direction

while true; do
    if [ "$(cat $PIN_PATH/value)" -eq 0 ]; then
        echo "Soft shutdown triggered"
        sudo shutdown -h now
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