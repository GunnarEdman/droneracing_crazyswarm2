# Crazyswarm 2 docker setup instructions
Instructions on how to setup ros2 workspace and crazyswarm2 by cloning cs2 from the github-repo and building from source-code (needed since cs2 is not added to apt in jazzy yet). Works on Linux and hopefully on mac.

## Setting up crazyswarm2 with docker
### Setup on host machine

Clone this repository to your local machine and navigate to the directory (e.g. droneracing_crazyswarm2)

```bash
git clone https://github.com/GunnarEdman/droneracing_crazyswarm2
cd droneracing_crazyswarm2
```

Navigate into src and clone the crazyswarm2 and motion capture tracking repos recursivly to ensure all dependencies are inlcuded. 

```bash
cd ros2_ws/src
git clone https://github.com/IMRCLab/crazyswarm2.git --recursive --depth 1
git clone https://github.com/IMRCLab/motion_capture_tracking.git --recursive --depth 1
```

### Container launch and acess

Navigate to repo root dir (with docker-compose file). Then build the Docker image AND launch a container in detached mode with one singel command.

```bash
cd ..
docker compose up -d --build
```

Enter the container in interactive mode using this command:
```bash
docker exec -it cs2 bash
```

### Setup crazyswarm2 inisde the workspace in the container

Once inside the container, execute the build script located in the workspace to install ROS dependencies and build Crazyswarm2.

```bash
/ws/cs2_build.sh
```

### Sourcing required inside container
Only this sourcing is required inside the container when working  with cs2:
```bash
source /ws/install/setup.bash
```

## Docker container commands
Some useful commands to work with the docker container using docker compose functionality.

```bash
# Exit the container
exit

# Stop the cotaienr temporarily (run on host)
docker compose stop

# "Decompose" the container (run on host in the same dir as the compose.yaml file, it stops it and removes the container cs2). Useful to run before composing a new image.
docker compose down
```

## Building custom packages
To build a specific packages in the workspace, for instance cs2_test packages, run:
```bash
colcon build --packages-select cs2_test --symlink-install
```