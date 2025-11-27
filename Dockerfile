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
    ros-dev-tools \
    ros-jazzy-joy \
    # Cfirmware setup for SIL drone sim
    gcc-arm-none-eabi \
    cmake \
    ninja-build \    
    # ---------------------------------
    && rm -rf /var/lib/apt/lists/*
    

# Create a venv and add env path
RUN python3 -m venv /opt/pyvenv
ENV PATH="/opt/pyvenv/bin:$PATH"

# Install python dependencies in the venv
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir \
    cflib\ 
    rowan\
    transforms3d\
    empy\
    lark\
    catkin_pkg\
    numpy\
    PyYAML \
    "nicegui < 3.0"
    # (<3.0 due to breaking changes in > 3.x)

# Cfirmware setup for SIL drone sim --------------------------------
WORKDIR /root

# Clone, configure and compile python bindings for crazyflie-firmware
RUN git clone --recursive --depth 1 https://github.com/bitcraze/crazyflie-firmware.git
WORKDIR /root/crazyflie-firmware
RUN make clean && \
    make cf2_defconfig && \
    make bindings_python PYTHON=python3

# Point Python to the compiled .so file we just built
ENV PYTHONPATH="${PYTHONPATH}:/root/crazyflie-firmware/build"
# -----------------------------------------------------------------

# Env configurations
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
    
# Set working directory
WORKDIR /ws

RUN rosdep update

# Source ROS and workspace setup.bash in root bashrc (this executes each time a new shell is started)
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi" >> /root/.bashrc

# Entrypoint (sources ROS setup on container start)
ENTRYPOINT ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && exec bash"]