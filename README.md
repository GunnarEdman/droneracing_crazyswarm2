# Crazyswarm 2 docker setup instructions
Instructions on how to setup ros2 workspace and crazyswarm2 by cloning cs2 from the github-repo and building from source-code (needed since cs2 is not added to apt in jazzy yet). Works on Linux and hopefully on mac.


## 1. Setup on Host System

Clone this repository to your local machine, navigate to the directory, and run the setup.sh script to clone the required ros2 dependency repos (Crazyswarm2 and Motion Capture Tracking)

```bash
cd droneracing
chmod +x setup.sh
./setup.sh
```

## 2. Container Launch

Build the Docker image and launch a container in detached mode.

```bash
docker compose up -d --build
```

## 3. Build Crazyswarm2

Enter the running container and execute the build script located in the workspace root to install dependencies and compile the project.

```bash
docker exec -it cs2 bash
./cs2_build.sh