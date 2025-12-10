# BYU Mars Rover GitHub Copilot Instructions

This repository contains the software for the BYU Mars Rover Capstone team's rover, built for the Mars Society's University Rover Challenge (URC). The codebase is primarily ROS2-based, running in Dockerized environments, with support for both simulation and physical hardware.

## Project Context

The BYU Mars Rover team is a student-led capstone project involving students from computer science, electrical engineering, and mechanical engineering. Many team members are new to ROS2, robotics, and rover development. Code should be:
- Clear and well-documented
- Easy to understand and maintain (high turnover rate)
- Consistent with existing coding conventions

**Before implementing features, always consult:**
- `README.md` for project overview and getting started
- `diagrams/` folder for architecture and system overviews
- Existing similar files for coding style and documentation patterns

## Repository Structure

```
marsrover_4.0/
├── .github/            # GitHub Actions for Docker image building
├── base_scripts/       # Base station launch scripts (rover_launch.sh, base_launch.sh, zed_launch.sh)
├── diagrams/           # Software architecture diagrams (CONSULT THESE OFTEN)
├── docker/             # Dockerfile and docker-compose.yaml for marsrover-ct container
├── firmware/           # PlatformIO workspaces for Arduino Mega and Nano
├── rover_ws/           # Main ROS2 workspace (PRIMARY DEVELOPMENT AREA)
│   └── src/            # ROS2 packages
├── scripts/            # Scripts for use inside marsrover-ct container
├── tutorial_ws/        # Empty ROS2 workspace for tutorials
└── zed_ws/             # ZED 2 camera ROS2 workspace
```

## Development Environment

### Docker Setup

The rover software runs in Docker containers:
- **Primary container**: `marsrover-ct` (contains most rover software)
- **Camera container**: `zed-ct` (ZED 2 camera software)

**CRITICAL**: Do NOT attempt to run these scripts yourself as an AI agent:
- `compose.sh` - Launches containers (user must run)
- `sim_launch.sh` - Launches Gazebo simulation (user must run from inside container's tmux)
- `base_launch.sh` - Base station launch
- `rover_launch.sh` - Rover launch
- `zed_launch.sh` - ZED camera launch

### Interacting with ROS2

To run ROS2 CLI commands, use `docker exec` to execute inside the `marsrover-ct` container:
```bash
docker exec -it marsrover-ct bash -c "source /home/ros/rover_ws/install/setup.bash && ros2 <command>"
```

## Build and Test Instructions

### Building ROS2 Workspace

From inside the `marsrover-ct` container:
```bash
cd ~/rover_ws
colcon build
source install/setup.bash
```

### Running Tests

```bash
cd ~/rover_ws
colcon test
colcon test-result --verbose
```

### Linting Python Code

Follow existing Python code style. Use descriptive docstrings with author and date information:
```python
"""
Brief description of the class/function

:author: Your Name
:date: Month Year
"""
```

## ROS2 Packages Overview

The `rover_ws/src/` directory contains:
- `rover_behaviors/` - Behavior tree implementations
- `rover_bringup/` - Launch files and configuration
- `rover_control/` - Drive control and muxing
- `rover_description/` - URDF robot descriptions
- `rover_gazebo/` - Gazebo simulation
- `rover_gui/` - User interfaces
- `rover_interfaces/` - Custom ROS2 message/service definitions
- `rover_localization/` - GPS and localization
- `rover_navigation/` - Nav2 integration
- `rover_perception/` - Camera and sensor processing

Consult `diagrams/` for detailed launch structure and software overviews.

## Key Technologies

- **ROS2 Humble** - Primary framework
- **Python 3** - Main language for nodes
- **C++** - Performance-critical nodes
- **Docker** - Containerized environments
- **Gazebo** - Simulation
- **Nav2** - Navigation stack
- **Fast DDS** - Discovery server for distributed systems
- **PlatformIO** - Arduino firmware development

## Network Configuration

The rover uses Fast DDS Discovery Server for communication between base station and rover:
- Discovery configs in `scripts/discovery/config/`
- Base station uses `base_super_client_config.xml`
- Rover uses `rover_super_client_config.xml`
- Default discovery port: 11811

## Coding Standards

### Python Nodes

1. Include comprehensive docstrings with:
   - Class/function description
   - Author name
   - Date (Month Year)
   - List of subscribers, publishers, services, actions

2. Use type hints where appropriate

3. Follow ROS2 naming conventions:
   - Topics: `snake_case`
   - Nodes: `snake_case`
   - Services: `snake_case`
   - Parameters: `snake_case`

4. Log appropriately:
   - `self.get_logger().info()` for important state changes
   - `self.get_logger().warn()` for recoverable issues
   - `self.get_logger().error()` for critical problems

### File Organization

- Launch files: `rover_bringup/launch/`
- Config files: `rover_bringup/config/`
- Custom messages: `rover_interfaces/msg/` or `rover_interfaces/srv/`

## Git Workflow

1. **Create a branch**: Name format `name/feature` (e.g., `spencer/drive-mux`)
2. **Make changes**: Develop incrementally with good documentation
3. **Rebase often**: Keep up-to-date with main branch
4. **Submit PR**: Get team review before merging

Main branch is protected - all changes must go through pull requests.

## Important Notes

- **Consult documentation first**: Always check `README.md` and `diagrams/` before implementing
- **Don't make assumptions**: Ask for clarification if unsure about requirements
- **Incremental development**: Build code step-by-step with user feedback
- **Test thoroughly**: Verify changes in both simulation and hardware when applicable
- **Document everything**: Future team members need to understand your code

## Common Patterns

### Creating a ROS2 Node

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    """
    Brief description
    
    :author: Your Name
    :date: Month Year
    
    Subscribers:
    - topic_name (Type)
    Publishers:
    - topic_name (Type)
    """
    
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info("MyNode started")

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Adding Dependencies

If you need to add system or Python dependencies:
1. Add them to `docker/Dockerfile`
2. Test building the image locally
3. Document the change in your PR
4. GitHub Actions will automatically build and push to DockerHub after merge

## Troubleshooting

- **ROS2 topics not visible**: Check Fast DDS discovery configuration, firewall settings
- **Build failures**: Ensure all dependencies are installed, check for syntax errors
- **Container issues**: Try rebuilding with `docker-compose build --no-cache`
- **Network issues**: Verify discovery server is running and accessible

## Additional Resources

- Project README: `/README.md`
- Architecture diagrams: `/diagrams/`
- ROS2 Humble docs: https://docs.ros.org/en/humble/
- Nav2 documentation: https://navigation.ros.org/
