FROM ros:jazzy-ros-base

# Install essential dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Common dependencies
    git\
    build-essential\
    python3-pip\
    python3-venv \ 
    python3-colcon-common-extensions\
    python3-rosdep \
    libusb-1.0-0-dev \
    libboost-all-dev \
    libpython3-dev \
    swig \
    ros-jazzy-tf-transformations \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Create a venv and add env path
RUN python3 -m venv /opt/pyvenv
ENV PATH="/opt/pyvenv/bin:$PATH"

# Install python dependencies in the venv
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir \
    cflib\ 
    rowan\
    nicegui\ 
    transforms3d\
    empy\
    lark\
    catkin_pkg\
    numpy\
    PyYAML

# Env configurations
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
WORKDIR /ws

RUN rosdep update    

# Entrypoint
ENTRYPOINT ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && exec bash"]






# ===============================================================================
# Not used stuff:
# ===============================================================================
# Add entrypoint script to the image and make it executable
# COPY entrypoint_build.sh /usr/local/bin/
# RUN chmod +x /usr/local/bin/entrypoint_build.sh

# # ENTRYPOINT script
# ENTRYPOINT ["/usr/local/bin/entrypoint_build.sh"]
# CMD ["bash"]


# Steps for cloning and building crazyswarm (done on host machine isntead to get a workspace)
# # Clone motion capture repo and all submodules (recursive)
# RUN git clone --depth 1 --recursive https://github.com/IMRCLab/motion_capture_tracking
# # RUN sudo apt-get install ros-jazzy-motion-capture-tracking

# # Clone the crazyswarm2 repository and all submodules (recursive)
# RUN git clone --depth 1 --recursive https://github.com/IMRCLab/crazyswarm2.git /ws/src/crazyswarm2

# Install ROS dependencies (uses the rosdep tool)
# RUN rosdep update \
# && rosdep install --from-paths /ws/src --ignore-src -y --rosdistro jazzy
# RUN rosdep update

#  # Build the workspace (including crazyswarm2)
# RUN /opt/pyvenv/bin/colcon build --merge-install --symlink-install