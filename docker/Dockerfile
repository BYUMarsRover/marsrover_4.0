# Created by Nelson Durrant, Oct 2024
FROM ros:humble-ros-base

# Why use ROS2 Humble?
#
# The simulator is built using Gazebo Classic, which is only supported in ROS2 Iron and older.
# Unfortunately, Nav2's simple GPS navigation functions are only supported in ROS2 Iron and newer,
# but there's a simple workaround on ROS2 Humble. The ZED's container is locked to ROS2 Humble by
# compatible L4T images (it has to match the Orin), and through a lot of trial and error we've learned
# that inter-Docker container FastDDS connections really only work reliably between matching distros.
# In addition, Iron has a sooner EOL date than Humble and less community support.
#
# In summary:
# .---------.---------------.---------------.---------------.---------------.
# |         | GZ Classic?   | Nav2 GPS?     | ZED Docker?   | EOL?          |
# |-------- |---------------|---------------|---------------|---------------|
# | Humble  | yes           | workaround    | yes           | May 2027      |
# | Iron    | yes           | yes           | no            | Dec 2024      |
# | Jazzy   | no            | yes           | no            | May 2029      |
# '---------'---------------'---------------'---------------'---------------'

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

# Define a username and password
ARG NAME=marsrover-docker
ARG PASS=thekillpack

# Create the user
RUN useradd -ms /bin/bash "$NAME" \
    && usermod -aG sudo,video,dialout "$NAME" \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && groupadd docker \
    && usermod -aG docker "$NAME" \
    && echo "$NAME:$PASS" | chpasswd

#################################
# IMPORTANT DOCUMENTATION NOTE! #
#################################

# When adding dependencies or Dockerfile commands below, please document your changes
# following this format:
#
#     # Why do we need this dependency or command? - Full Name, Month Year
#     < Add system dependencies here >
#
# https://docs.docker.com/reference/dockerfile/

# Install system dependencies
RUN apt update && apt upgrade -y --no-install-recommends \
    && apt install -y --no-install-recommends \
    # Install basic system utilities - Nelson Durrant, Oct 2024
    curl wget git vim nano tmux python3-pip python3-venv \
    # Install graphical utilities - Nelson Durrant, Oct 2024
    x11-apps x11-utils x11-xserver-utils xauth \
    # Terrain loading and processing packages - Nelson Durrant, July 2025
    libgdal-dev python3-rasterio \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Pre-built full navigation and localization stack - Nelson Durrant, Feb 2025
    ros-humble-navigation2 ros-humble-robot-localization \
    # Used for GUIs - Nelson Durrant, Feb 2025
    ros-humble-rviz2 ros-humble-rqt* python3-tk \
    # Tool for plotting and visualizing data from ROS2 bags - Nelson Durrant, Feb 2025
    ros-humble-plotjuggler-ros \
    # Used for detecting ArUco markers in sim and real world - Nelson Durrant, Feb 2025
    ros-humble-aruco-opencv \
    # Controlling the robot's movement through a joystick or keyboard - Nelson Durrant, Feb 2025
    ros-humble-joy ros-humble-teleop-twist-joy ros-humble-teleop-twist-keyboard ros-humble-topic-tools \
    # ROS2 and tf2 tutorial installs - Nelson Durrant, Feb 2025
    ros-humble-turtle-tf2-py ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-turtlesim \
    # URDF tools and launch configs - Nelson Durrant, Mar 2025
    ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-urdf-launch \
    # Official ROS2 interfaces for the ZED camera - Nelson Durrant, Mar 2025
    ros-humble-zed-msgs \
    # Open-source robotic arm manipulation platform - Nelson Durrant, Jun 2025
    ros-humble-moveit \
    # Joy package for science control - Jacob Gunnell, Sept 2025
    ros-humble-joy-linux \
    && rm -rf /var/lib/apt/lists/*

# Install architecture-specific ROS packages
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
    # Simulation packages - Nelson Durrant, Feb 2025
    ros-humble-nav2-bringup ros-humble-turtlebot3-gazebo \
    # Tools for visualizing the robot's location - Nelson Durrant, Feb 2025
    ros-humble-mapviz ros-humble-mapviz-plugins ros-humble-tile-map \
    && rm -rf /var/lib/apt/lists/*; \
    fi

USER $NAME
WORKDIR /home/$NAME

# Install python dependencies
RUN pip3 install --no-cache-dir \
    # Tool for converting GPS coordinates to UTM and vice versa - Nelson Durrant, Feb 2025
    utm \
    # Used for the Mars Rover ASCII art display on startup - Nelson Durrant, Mar 2025
    --upgrade pillow asciimatics \
    # Used for launch script tmux window management - Nelson Durrant, Mar 2025
    tmuxp \
    # Used for serial communication with the Arduinos - Nelson Durrant, Mar 2025
    pyserial \
    # Used for pre-caching map tiles for rover test locations - Nelson Durrant, Mar 2025
    mercantile requests \
    # Simple A* path planning algorithm implementation - Nelson Durrant, Apr 2025
    astar \
    # Used for experimental MCP server hookup - Nelson Durrant, July 2025
    fastmcp uv
ENV PATH="$PATH:/home/$NAME/.local/bin"

# Install PlatformIO for firmware development - Nelson Durrant, Mar 2025
RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    && python3 get-platformio.py \
    && rm get-platformio.py

# Set up PlatformIO CLI command symlinks - Nelson Durrant, Mar 2025
USER root
RUN mkdir -p /home/$NAME/.platformio/penv/bin \
    && ln -s /home/$NAME/.platformio/penv/bin/platformio /usr/local/bin/platformio \
    && ln -s /home/$NAME/.platformio/penv/bin/pio /usr/local/bin/pio \
    && ln -s /home/$NAME/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb
USER $NAME

# Install PlatformIO libraries - Nelson Durrant, Mar 2025
COPY --chown=$NAME:$NAME firmware /home/$NAME/firmware
RUN pio pkg install --project-dir /home/$NAME/firmware/mega_pio \
    && pio pkg install --project-dir /home/$NAME/firmware/nano_pio \
    && pio pkg install --project-dir /home/$NAME/firmware/nano_science_pio \
    && rm -rf /home/$NAME/firmware

# Set up automatic ROS2 sourcing and colorized output - Nelson Durrant, Oct 2024
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc \
    && touch ~/.hushlogin

# Set tmux to use color - Jacob Gunnell, Sept 2025
RUN echo "set -g default-terminal \"screen-256color\"" >> ~/.tmux.conf

CMD ["bash"]
