# Xbox Controller Guide for Mars Rover Teleoperation

## Controller Layout

```
                    ╔════════════════════════════════════╗
                    ║                                    ║
         [BACK]     ║          XBOX CONTROLLER           ║     [START]
           (6)      ║                                    ║       (7)
                    ║                                    ║
                    ║        ┌───┐        ┌───┐         ║
                    ║    ┌───┤ Y ├───┐    │   │         ║
    Left Stick      ║    │ X ├───┤ B │    │   │         ║  Right Stick
    (Unused)        ║    └───┤ A ├───┘    │   │         ║  (Turn Left/Right)
                    ║        └───┘        └───┘         ║
       ○            ║                                    ║         ○
      /│\           ║                                    ║        /│\
       │            ║     [LT]          [RT]             ║         │
     Axis 0,1       ║      (2)          (5)              ║      Axis 3,4
                    ║                                    ║
                    ║    [LB]            [RB]            ║
                    ║                                    ║
                    ╚════════════════════════════════════╝
```

## Manual Driving Controls

### Mode Switching
- **START Button** → Switch to **Manual (Teleop) Mode**
  - LED turns **BLUE**
  - You can now control the rover manually
  
- **BACK Button** → Switch to **Auto Mode**
  - LED turns **GREEN** (flashing)
  - Autonomy system takes control

### Driving (When in Manual Mode)

#### Forward/Backward Movement
- **Left Trigger (LT)** - Axis 4
  - Push down = Move forward
  - Release = Stop
  - *Speed: 0.5 (half speed)*

#### Turning
- **Right Stick (Horizontal)** - Axis 3
  - Push right = Turn right
  - Push left = Turn left
  - Center = Go straight
  - *Speed: 0.5 (half rotation speed)*

#### Turbo Mode
- **Right Bumper (RB)** - Button 5
  - Hold while using triggers/stick = **2x speed**
  - *Forward/turning speed: 1.0 (full speed)*

### D-Pad (Elevator Control - When Implemented)
- **D-Pad Up** = Elevator up
- **D-Pad Down** = Elevator down
- **D-Pad Left/Right** = Elevator speed adjustment

## Quick Reference

| Control | Function | Notes |
|---------|----------|-------|
| **START** | Manual Mode | LED: Blue |
| **BACK** | Auto Mode | LED: Green (flashing) |
| **LT** (Axis 4) | Forward/Back | Speed: 0.5 |
| **Right Stick ↔** (Axis 3) | Turn L/R | Speed: 0.5 |
| **RB** | Turbo Boost | 2x speed when held |
| Left Stick | *Not Used* | Reserved |
| A/B/X/Y | *Not Used* | Reserved |

## LED Status Indicators

The rover's LED indicates current mode:

- 🔵 **Blue (Solid)** = Manual/Teleop Mode - You have control
- 🟢 **Green (Flashing)** = Auto Mode - Autonomy is driving
- 🔴 **Red (Solid)** = Arrival Mode - Rover reached destination

## Tips for Operators

1. **Always check LED color** before assuming you have control
2. Press **START** to take manual control if needed
3. Press **BACK** to return control to autonomy
4. Use **turbo sparingly** - half speed (0.5) is often safer
5. **Release LT completely** to stop - there's no brake button

## Troubleshooting

**Rover not responding to controller?**
1. Check LED - make sure it's BLUE (manual mode)
2. Press START button to switch to manual mode
3. Verify controller is connected to base station
4. Check that base_autonomy.launch is running

**Can't switch modes?**
- Ensure rover is connected to base station network
- Check that rover_control.launch is running on the rover
- Verify ROS2 Discovery Server is running (192.168.1.120:11811)
