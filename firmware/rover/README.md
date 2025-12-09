# Rover Hardware Scripts

This directory contains scripts for configuring hardware on the physical rover.

## Soft Shutdown Installation

The `install_soft_shutdown_switch.sh` script sets up a systemd service that monitors GPIO pin 7 for a soft shutdown trigger. This allows for graceful shutdown of the rover using a physical button or switch.

### Features

- Monitors GPIO pin 7 (which is pull-down by default on the Jetson)
- Triggers a safe system shutdown when pin 7 goes high
- Runs as a systemd service that starts automatically on boot
- Automatically restarts if the monitoring process fails

### Installation

Run the installation script on the rover:

```bash
sudo bash install_soft_shutdown_switch.sh
```

This script will:
1. Create `/usr/local/bin/soft_shutdown.sh` - the monitoring script
2. Create `/etc/systemd/system/soft_shutdown.service` - the systemd service definition
3. Enable and start the service

### Hardware Setup

Connect a momentary push button or toggle switch between:
- GPIO pin 7 (physical pin 26 on Jetson Orin)
- 3.3V power (or 5V through appropriate resistor)

When the button is pressed or switch is closed, pin 7 will go high and trigger a shutdown.

### Service Management

Check service status:
```bash
systemctl status soft_shutdown.service
```

View service logs:
```bash
journalctl -u soft_shutdown.service -f
```

Stop the service:
```bash
sudo systemctl stop soft_shutdown.service
```

Restart the service:
```bash
sudo systemctl restart soft_shutdown.service
```

Disable automatic startup:
```bash
sudo systemctl disable soft_shutdown.service
```

### Notes

- Pin 7 is configured as pull-down by default, meaning it reads low (0) when nothing is connected
- The service monitors for the pin to go high (1), which happens when the switch connects it to power
- The service will automatically restart if it crashes, with a 10-second delay between restart attempts
