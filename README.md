[Get Started](https://github.com/BYUMarsRover/marsrover_4.0?tab=readme-ov-file#get-started)

[Essential Tutorials](https://github.com/BYUMarsRover/marsrover_4.0?tab=readme-ov-file#essential-tutorials)

[How-To Videos](https://github.com/BYUMarsRover/marsrover_4.0?tab=readme-ov-file#how-to-videos)

[Software Diagrams](https://github.com/BYUMarsRover/marsrover_4.0?tab=readme-ov-file#software-diagrams)

[Contributing](https://github.com/BYUMarsRover/marsrover_4.0?tab=readme-ov-file#contributing)

--

> **TO THE 2026 MARS ROVER TEAM:** I've helped with writing a decent amount of robot software from scratch here at BYU, and oftentimes have been part of a team that runs up against some architecture-based limitations later on (usually too integrated into our software at that point to change) that we really wished we would have understood better before we started. With the shift from ROS1 to ROS2 in the 2024-2025 Capstone year, I saw an opportunity to prevent some of those future problems and put in 30+ hours of research on robotics development best practices at the beginning of the Winter 2025 semester -- and learned A TON about the open-source tools and software structure professional robotic projects use. After a short coding spree, this repository is the synthesis of our current approach with some of those best practices, including:
> - A Dockerized full-mission Gazebo simulator for rapid development and validation
> - Industry-standard Nav2 dynamic path planning and hazard avoidance
> - Tuned and tested EKF for fusing odometry sources with GPS data
> - A fully-defined map->odom->robot TF tree and rover URDF
> - LLM integration using MCP for natural language control of the rover
> - Lots and lots of QOL and scripting updates
> 
> I've successfully completed GPS, aruco, and object detection legs with this code running on the rover just behind the EB (even navigating between and around the trees and posts!). I've probably completed hundreds more in simulation, testing outlier cases like blocked hex points. However, when I've tried to increase the rover speed on our physical hardware to as quick as we'd need to run it at competition, the CPU can't keep up with the depth cloud processing from the ZED camera for object avoidance and we don't recognize obstacles in time before crashing into them. Even with CPU cost-cutting measures, it's clear that we simply just don't have the processing power on the Jetson Orin to run Nav2 hazard avoidance at the speed we need to to compete. We need a more powerful computer -- a laptop with a GPU or similar -- or another SBC in addition to the Orin. (Admittedly, I haven't had enough rover time to test mapping the point cloud to a laser scan or just using a LiDAR sensor instead to lower the costmap computation load, but I don't think we should be so close to the CPU limit regardless.)
>
> Due to the needed structural changes, this code was written mostly from scratch and doesn't share a lot of similarities with the Autonomy-ROS2 repo. It definitely is a marked shift in a new direction. <mark>In short, however, I think there's clear 100-point potential in this approach and these open-source tools. I'd encourage the 2026 team to [mess around with the simulator to understand a bit better how it all works](https://youtu.be/ZFADRaRCdOU), run some tests on the rover, consider investing in a more powerful computing stack, and then revisit development on this code base.</mark>
>
> I'll be starting my Master's program at BYU in Fall 2025, but would be more than happy to help answer any questions or help you get started with it. One small step for a rover, one giant leap for roverkind!
>
> Nelson Durrant
>
> --
>
> **MAY 2025 UPDATE:** We actually acquired a laptop with a GPU a couple of weeks before competition! However, we determined our team's familiarity with debugging and working with the previous software was more of an asset than the advantages this new stack would offer for the 2025 competition.

--

### Get Started

![ASCII Art](https://github.com/user-attachments/assets/18a7825f-216c-4df9-b4f3-7c91c4bf0153)

> **NOTE:** Newer Macs that use ARM64 architecture (M1, M2, etc) are not able to run the Gazebo simulation due to missing package support, and have not been extensively tested.

**Windows:**

- Install WSL2 on your Windows machine by following the instructions [here](https://docs.microsoft.com/en-us/windows/wsl/install).

- Install Docker Desktop on your Windows machine by following the instructions [here](https://docs.docker.com/desktop/), and enable the WSL2 backend by following the instructions [here](https://docs.docker.com/desktop/windows/wsl/).

- Open a WSL2 terminal and clone the marsrover_4.0 repo into your WSL2 environment using `git clone https://github.com/BYUMarsRover/marsrover_4.0.git`.

- Run `cd marsrover_4.0 && bash compose.sh` to pull and launch the latest Docker image from DockerHub.

**Linux:**

- Install Docker Engine on your Linux machine by following the instructions [here](https://docs.docker.com/engine/install/ubuntu/). Make sure to follow the post-installation steps to allow your user to run Docker commands without `sudo`.

- Open a terminal and clone the marsrover_4.0 repo into your Linux environment using `git clone https://github.com/BYUMarsRover/marsrover_4.0.git`.

- Run `cd marsrover_4.0 && bash compose.sh` to pull and launch the latest Docker image from DockerHub.

--

### Essential Tutorials

> **NOTE:** <mark>We would **strongly encourage** each year's team to take a couple of months at the beginning of the first semester and simply work together through these tutorials before diving into software development.</mark> It may not seem romantic, but I promise it'll be worth it.

**General:**

[Linux CLI Tutorial](https://linuxjourney.com/lesson/the-shell)

[GitHub Basics Tutorial](https://docs.github.com/en/get-started/start-your-journey/hello-world)

[Docker Concepts and Tutorials](https://docs.docker.com/get-started/introduction/whats-next/)

[ROS2 Concepts](https://docs.ros.org/en/humble/Concepts/Basic.html)

[ROS2 CLI Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)*

[ROS2 Code Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)*

[ROS2 Discovery Server Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html)*

[Robotics in ROS2 Tutorial](https://github.com/henki-robotics/robotics_essentials_ros2/tree/main)

**Autonomy:**

[Nav2 Concepts](https://docs.nav2.org/concepts/index.html)

[Nav2 Tutorials (Gazebo Classic)](https://docs.nav2.org/setup_guides/index.html)*

[GPS Navigation w Nav2 Tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)

**Servicing:**

[MoveIt2 Concepts](https://moveit.picknik.ai/humble/doc/concepts/concepts.html)

[MoveIt2 Tutorials](https://moveit.picknik.ai/humble/doc/tutorials/tutorials.html)*

> **NOTE:** Although MoveIt2 is preinstalled, the standard MoveIt2 tutorials require removing the Docker container's MoveIt2 installation and rebuilding it from source using colcon.

**All of the dependencies for these tutorials are preinstalled in the Docker container, and we've mounted 'tutorial_ws' as a dedicated ROS2 tutorial workspace.*

--

### How-To Videos

[Running the Simulation](https://youtu.be/ZFADRaRCdOU)

![Running the Simulation](https://github.com/user-attachments/assets/fb015c0e-7106-4704-8ae2-223fffd4caff)

[Localization Debugging](https://youtu.be/X4Mq1sxr7JY)

![Localization Debugging](https://github.com/user-attachments/assets/9eb8dd06-bbb7-4d36-8745-458301bda33f)

[LLM Control with MCP](https://youtu.be/rluqQbQE0hE)

![LLM_Control with MCP](https://github.com/user-attachments/assets/b066e76d-600f-4168-be79-4cf39f913346)

--

### Software Diagrams

> **NOTE:** To make changes to a diagram, edit the code in the corresponding `diagrams/*/*.txt` file in the repository and visit [this website](https://www.mermaidchart.com/) to generate a new PNG file.

**Autonomy:**

[Software Overview](diagrams/autonomy/software_overview.png)

[Launch Structure](diagrams/autonomy/launch_structure.png)

[State Machine](diagrams/autonomy/state_machine.png)

[Transform Tree](diagrams/autonomy/transform_tree.pdf)

[Sim Software Overview](diagrams/autonomy/simulation/sim_software_overview.png)

[Sim Launch Structure](diagrams/autonomy/simulation/sim_launch_structure.png)

> **TODO:** Add Servicing, Delivery, and Science software diagrams here.

--

### Contributing

- **Create a new branch.** This repository's main branch is protected, so you'll need to create a new branch. Name your branch with a combination of your name and the feature you are working on (e.g. nelson/repo-docs).

- **Make your changes.** Develop and debug your new feature or bug fix. Add good documentation.

    > **NOTE:** If you need to add dependencies, add them to the Dockerfile in your branch and test building the image locally. Once your pull request is merged into main, the GitHub CI workflow will automatically build and push the new Docker image to DockerHub.

- **Rebase your branch often.** Keep your branch up-to-date with main by rebasing. This will help prevent merge conflicts down the road.

- **Submit a pull request.** Once you have made and tested your changes, create a new pull request. Get another member of the team to review and approve it, and you can merge your new code into the main branch.

--

Created by Nelson Durrant, Feb 2025.
