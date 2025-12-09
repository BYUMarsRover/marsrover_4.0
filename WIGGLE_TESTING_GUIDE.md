# Wiggle Integration - Testing Guide

## Overview

This guide explains how to build and test the new wiggle recovery behavior that has been integrated into the Mars Rover navigation stack.

## What is the Wiggle Behavior?

The wiggle behavior is a Nav2 recovery plugin that performs oscillating forward-backward movements. It's designed to help the rover:
- Get unstuck from soft terrain or obstacles
- Improve localization by collecting sensor data from multiple positions
- Clear minor obstructions through rocking movements

## Building the Code

The wiggle behavior will be built when you build the rover workspace inside the Docker container:

1. Launch the Docker container (if not already running):
   ```bash
   ./compose.sh
   ```

2. Inside the container, build the rover workspace:
   ```bash
   cd ~/rover_ws
   colcon build --packages-select rover_behaviors
   source install/setup.bash
   ```

## Testing the Wiggle Behavior

### Option 1: Test in Simulation

1. Launch the full simulation environment (run this inside your tmux session):
   ```bash
   sim_launch.sh
   ```

2. In a separate terminal, you can manually trigger the wiggle behavior:
   ```bash
   ros2 action send_goal /wiggle nav2_msgs/action/BackUp "{target: {x: 0.0, y: 0.0, z: 0.0}, speed: 0.3, time_allowance: {sec: 30, nanosec: 0}}"
   ```

### Option 2: Test with Behavior Tree

The wiggle behavior is registered with Nav2's behavior server and can be called from behavior trees. You can add it to a custom behavior tree XML file or call it programmatically.

### Option 3: Test on Physical Rover

Once you've verified the behavior works in simulation, you can test it on the physical rover:

1. Launch the rover software using the appropriate launch script
2. Monitor the behavior by watching the rover's movements
3. Check the logs for wiggle behavior output:
   ```bash
   ros2 topic echo /rosout | grep -i wiggle
   ```

## Configuration Parameters

You can adjust the wiggle behavior parameters in `rover_ws/src/rover_navigation/config/navigation_params.yaml`:

```yaml
wiggle:
  plugin: "rover_behaviors/Wiggle"
  wiggle_cycles: 3          # Number of forward-backward oscillations
  wiggle_distance: 0.3      # Distance in meters for each movement
  max_linear_vel: 0.5       # Maximum linear velocity (m/s)
  min_linear_vel: 0.1       # Minimum linear velocity (m/s)
  linear_acc_lim: 2.5       # Linear acceleration limit (m/s²)
```

After changing parameters:
1. Rebuild the workspace: `colcon build --packages-select rover_navigation`
2. Source the setup: `source install/setup.bash`
3. Restart the navigation stack

## Expected Behavior

When the wiggle behavior is triggered, you should observe:
1. The rover moving backward a short distance (default 0.3m)
2. The rover moving forward the same distance
3. This pattern repeating for the configured number of cycles (default 3)
4. The rover stopping after completing all cycles

## Troubleshooting

### Build Errors

If you encounter build errors:
1. Make sure you're inside the Docker container
2. Check that all dependencies are installed (they should be in the Docker image)
3. Try a clean build: `rm -rf build install log && colcon build`

### Runtime Errors

If the behavior doesn't work as expected:
1. Check that the behavior server is running: `ros2 node list | grep behavior_server`
2. Verify the plugin is loaded: `ros2 param get /behavior_server behavior_plugins`
3. Check the logs for error messages: `ros2 topic echo /rosout`

### Testing Tips

- Start with small wiggle_distance values (0.2-0.3m) for safety
- Test in simulation before trying on the physical rover
- Make sure you have enough space for the rover to move in both directions
- Monitor the rover closely during the first few physical tests

## Integration with Nav2

The wiggle behavior can be integrated into Nav2's behavior trees for automatic recovery. For example, you could add it to a recovery sequence when the rover gets stuck.

Example behavior tree snippet (not yet implemented):
```xml
<RecoveryNode number_of_retries="3">
  <Sequence>
    <ClearCostmap/>
    <Wiggle/>
    <Wait wait_duration="2"/>
  </Sequence>
</RecoveryNode>
```

## Further Reading

- [Nav2 Behavior Plugins Documentation](https://docs.nav2.org/plugin_tutorials/docs/writing_new_behavior_plugin.html)
- [rover_behaviors/README.md](rover_ws/src/rover_behaviors/README.md) - Detailed behavior documentation
- [Nav2 Concepts](https://docs.nav2.org/concepts/index.html)

## Questions or Issues?

If you encounter any problems or have questions about the wiggle behavior:
1. Check the rover_behaviors/README.md for detailed documentation
2. Review the implementation in wiggle.hpp and wiggle.cpp
3. Consult the Nav2 documentation for behavior plugin details
4. Contact the team member who implemented this feature

---

Created as part of wiggle_integration PR, Dec 2025
