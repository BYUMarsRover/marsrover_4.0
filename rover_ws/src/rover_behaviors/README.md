# rover_behaviors

Custom Nav2 recovery behaviors for the BYU Mars Rover.

## Overview

This package provides custom behavior plugins for Nav2 that are specifically designed for the Mars Rover's navigation stack. These behaviors help the rover handle challenging terrain and recovery situations during autonomous navigation.

## Behaviors

### CustomSpin

A modified version of Nav2's default Spin behavior with collision detection disabled.

**Purpose:** Allows the rover to spin in place without false collision detections that were frequently occurring during real-world testing.

**Usage:** This behavior is automatically used by Nav2's behavior trees for in-place rotation maneuvers.

**Parameters:**
- `simulate_ahead_time` (default: 2.0) - Time in seconds to simulate ahead
- `max_rotational_vel` (default: 1.0) - Maximum rotational velocity in rad/s
- `min_rotational_vel` (default: 0.4) - Minimum rotational velocity in rad/s
- `rotational_acc_lim` (default: 3.2) - Rotational acceleration limit in rad/s²

**Author:** Nelson Durrant, Apr 2025

**Reference:** Based on [nav2_behaviors/Spin](https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/plugins/spin.cpp)

### Wiggle

A recovery behavior that performs oscillating forward-backward movements to help the rover get unstuck.

**Purpose:** When the rover becomes stuck or needs to improve its localization, the wiggle behavior performs a series of forward and backward movements in an oscillating pattern. This can help:
- Free the rover from soft terrain or obstacles
- Improve localization by collecting more sensor data from different positions
- Clear minor obstructions by rocking the rover

**Usage:** Can be called as a recovery behavior through Nav2's behavior tree system or triggered manually for testing.

**Parameters:**
- `wiggle_cycles` (default: 3) - Number of complete forward-backward oscillations to perform
- `wiggle_distance` (default: 0.3) - Distance in meters for each forward/backward movement
- `max_linear_vel` (default: 0.5) - Maximum linear velocity in m/s
- `min_linear_vel` (default: 0.1) - Minimum linear velocity in m/s
- `linear_acc_lim` (default: 2.5) - Linear acceleration limit in m/s²
- `simulate_ahead_time` (default: 2.0) - Time in seconds to simulate ahead

**Author:** Created for wiggle_integration, Dec 2025

**Reference:** Based on [nav2_behaviors/BackUp](https://github.com/ros-navigation/navigation2/blob/humble/nav2_behaviors/plugins/back_up.cpp)

**Note:** Similar to CustomSpin, collision checking is disabled to avoid false positives that would prevent the recovery behavior from functioning properly.

## Configuration

The behaviors are configured in `rover_navigation/config/navigation_params.yaml` under the `behavior_server` section.

Example configuration:
```yaml
behavior_server:
  ros__parameters:
    behavior_plugins:
      ["spin", "backup", "wiggle", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "rover_behaviors/CustomSpin"
    wiggle:
      plugin: "rover_behaviors/Wiggle"
      wiggle_cycles: 3
      wiggle_distance: 0.3
      max_linear_vel: 0.5
      min_linear_vel: 0.1
      linear_acc_lim: 2.5
```

## Building

This package is part of the main rover workspace and is built automatically when building the `rover_ws` workspace:

```bash
cd ~/rover_ws
colcon build --packages-select rover_behaviors
```

## Testing

To test the wiggle behavior in isolation using the ROS2 action interface:

```bash
# Make sure the behavior server is running
ros2 run nav2_behaviors behavior_server

# Call the wiggle action
ros2 action send_goal /wiggle nav2_msgs/action/BackUp "{target: {x: 0.0, y: 0.0, z: 0.0}, speed: 0.3, time_allowance: {sec: 30, nanosec: 0}}"
```

## License

Apache License 2.0 - See LICENSE file for details.

## References

- [Nav2 Behavior Plugins Documentation](https://docs.nav2.org/plugin_tutorials/docs/writing_new_behavior_plugin.html)
- [Nav2 Behaviors Package](https://github.com/ros-navigation/navigation2/tree/humble/nav2_behaviors)
