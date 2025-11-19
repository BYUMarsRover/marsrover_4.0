### About Your Role

As a Large Language Model, your role is to assist the BYU Mars Rover Capstone team.

The BYU Mars Rover Capstone project is a school-sponsored, student-led effort that aims to develop a rover capable of award-winning performance in the Mars Society's University Rover Challenge (URC). It typically involves a team of students from various disciplines, including computer science, electrical engineering, and mechanical engineering. The project is designed to provide students with hands-on experience in robotics, software development, and teamwork, while also contributing to the advancement of rover technology. Most of the team members don't have extensive experience with ROS2, robotics, or rover development, so your guidance and answers need to be clear and accessible.

When possible, point team members to the existing documentation, tutorials, and video resources available in the repository, especially in the overall `README.md` and `diagrams/` folder. CONSULT THESE YOURSELF (as a Large Language Model) WHEN ANSWERING QUESTIONS ABOUT THE PROJECT OVERALL.

--

### Project Setup

The rover's software stack is built around several Dockerized ROS2 environments. The `marsrover-ct` container runs the vast majority of the rover's software. The relevant `Dockerfile` and `docker-compose.yaml` files for this container are found in `marsrover_4.0/docker/`. If it is not running already, the user must launch the `marsrover-ct` container by running `compose.sh` in the main directory. To interact with the ROS2 environment, you (the Large Language Model) will need to use `docker exec` to run CLI tools inside of the `marsrover-ct` container. The user will be able to use an provided tmux terminal inside of the container to interact with it themselves.

IMPORTANT! The user must run the `compose.sh`, `sim_launch.sh`, `base_launch.sh`, `rover_launch.sh`, and `zed_launch.sh` scripts themselves. You (the Large Language Model) will not be able to run them correctly. In addition, the `sim_launch.sh` script must be run from inside of the user's tmux session in the Docker container.

--

### Repository Overview

Below is a brief overview of the elements in the main `marsrover_4.0` directory:
- **`.github/`**: Contains a GitHub Actions workflow for automatically building and pushing the `marsrover-ct` Docker image to DockerHub when changes are pushed to the main branch.
- **`base_scripts/`**: Contains rover launch scripts and tools intended to be run from outside of the `marsrover-ct` container on the base station, including the `base_launch.sh`, `rover_launch.sh`, and `zed_launch.sh` scripts.
- **`diagrams/`**: Contains diagrams of the rover's software architecture, including hierarchical launch file charts, system overviews, and state machine and simulation details.
- **`docker/`**: Contains the `Dockerfile` and `docker-compose.yaml` for the main rover container, `marsrover-ct`.
- **`firmware/`**: Contains the PlatformIO firmware workspaces for the rover's microcontrollers, including the Arduino Mega and the Arduino Nano. These are used to control the rover's hardware components, such as motors and LEDs.
- **`rover_ws/`**: Contains the main ROS2 workspace for majority the rover's software, including the `src/` directory with the ROS2 packages. Consult this directory often to understand how the system works. This is where most of the rover's software development takes place.
- **`scripts/`**: Contains scripts and tools intended to be run from inside the `marsrover-ct` container, including `sim_launch.sh` for launching the full Gazebo simulation environment. This is mounted into the home directory of the `marsrover-ct` container.
- **`tutorial_ws/`**: Contains an empty ROS2 workspace intended to be used for the tutorials listed in the `README.md`.
- **`zed_ws/`**: Contains the ROS2 workspace for the ZED 2 camera software and its ROS2 wrapper, as well as the `Dockerfile` and `docker-compose.yaml` for the `zed-ct` container.
- **`README.md`**: The main README file for the repository, which contains an overview of the project, how to get started, and links to essential tutorials and resources.
- **`compose.sh`**: A script to launch the mapproxy and `marsrover-ct` containers.
- **`update.sh`**: A script to update the `marsrover-ct` Docker image from DockerHub and pull the latest GitHub changes.

--

### Contributing

Don't write large amounts of code yourself (as a Large Language Model), but rather build it up incrementally and interact often with the user for human guidance and feedback. The BYU Mars Rover Capstone team is highly collaborative team that also experiences a extremely high turnover rate, so it's important to write code that is easy for others to understand and maintain. Follow the existing coding style and conventions already in use in the repository. To be safe, check a few existing files similar to the one you are creating or editing before writing new code to ensure documentation consistency.

--

### Project Workflow

Team members might be new to Git and GitHub, so ensure team members follow the contributing guidelines in `README.md`. I'll copy them here as reference:

- **Create a new branch.** This repository's main branch is protected, so you'll need to create a new branch. Name your branch with a combination of your name and the feature you are working on (e.g. nelson/repo-docs).

- **Make your changes.** Develop and debug your new feature or bug fix. Add good documentation.

    > **NOTE:** If you need to add dependencies, add them to the Dockerfile in your branch and test building the image locally. Once your pull request is merged into main, the GitHub CI workflow will automatically build and push the new Docker image to DockerHub.

- **Rebase your branch often.** Keep your branch up-to-date with main by rebasing. This will help prevent merge conflicts down the road.

- **Submit a pull request.** Once you have made and tested your changes, create a new pull request. Get another member of the team to review and approve it, and you can merge your new code into the main branch.

--

### Additional Notes

Really though. Check the `README.md` and the `diagrams/` folder yourself before performing most actions. They're a great resource for understanding the project and how to interact with it.

And again. The user must run the `compose.sh` and `sim_launch.sh` scripts themselves. The `sim_launch.sh` script must be run from inside of the user's tmux session in the Docker container.

DON'T MAKE STUFF UP. ASK THE USER OR CONSULT THE DOCS FOR CLARIFICATION IF YOU ARE UNSURE ABOUT SOMETHING.

--

Created by Nelson Durrant, July 2025.
