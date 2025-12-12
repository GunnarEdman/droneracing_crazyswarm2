# Instructions for when connectign to QTM in the lab

## QTM computer general setup
- Follow the setup that big daddy slim did
- Very important seps for using active marker deck on crazyflie:
    - Set to active marker mode
    - Ensure 450 us exposure is set (maybe not needed, ask big daddy slim)
    - Create rigid body and most importantly in this list, ENSURE IT HAS THE SAME NAME AS THE DRONE IN ```crazyflies.yaml```, that is ```cf231```

### QTM computer - Potential problems
- Windows firewall nuking QTM communications
- Static IP need to be set for the network card connected via ethernet to the camera network
- Disablign all stupid options in the network adapter settigns (advenced tab) such as:
    - Flow-control - disable
    - Jumbo packet size should be changed to biggest avaible (4 KB for the Ugreen usb-ethernet adapter)
    - Green internet - disable
    - Interrupt Moderation - disable if avaible
    - Receive Buffers - Set to Maximum (e.g. 2048) if available.

- Not all cameras attached to the network (should work, but failed for us)
- Try rebooting cameras with reboot fucntionality in QTM software (check QTM docs)

-------------------------------------------------------

## CS2 computer

### Highlights from what Gunnar did 2025-12-12 to get it working 
(how to get QTM data to publish to poses topic in cs2 inside container)

- First obvious step is to connect to the same private network which the QTM PC is connected to!!!

- If you have a Linux firewall configured, disabled it as a sanity check (for isntance ```sudop ufw disable```).

- Network mode used in the container is unknown right now, either host or specific port bridging were used (unsure which were used). Try host mode first, if it fails comment out ``` network_mode: "host"``` and add these port bridges:
    ```bash
    ports:
        - "8080:8080"           # NiceGUI
        - "22223:22223"         # QTM Command (TCP)
        - "22222:22222"         # QTM v1.0 (Legacy) - FALLBACK
        - "6734:6734/udp"       # QTM Data (UDP)
    ```

- Ensure missing dependency submoudles are present for mocap_capture_tracking \
    Run this on host machine:
    ```bash
    # Ensure mocap dependencies (command I used on my own machine, adapt path to your own)
    cd /home/<user_name>/droneracing/ros2_ws/src/motion_capture_tracking/
    git submodule update --init --recursive
    ```

- Build workspace (run insde container)
    ```bash
    bash cs2_build.sh
    ```

    If a LOTS of errors occour (3-4 std errors are expected, and warnigns are expected don't worry about them) and build fails before finishing with all , one thing to try is to wipe the old buidl (this measn rebuild all of cs2, mocap and custom packeges, will take 4 or more minutes...):

    ```bash
    rm -rf build/ install/ log/
    ```

- Add qualisys and IP-adress of QTM config files in cs2 and motion_capture_tracking repo. In crazyflies.yaml:
    ```yaml
    # named list of all robots
    fileversion: 3

    robots:
    cf231:
        enabled: true
        uri: radio://0/43/2M/E7E7E7E7E7
        initial_position: [0.0, 0.0, 0.0]
        type: cf21_mocap_deck  # see robot_types

    # Keep default stuff here...
    
    robot_types:
        cf21_mocap_deck:
            motion_capture:
            tracking: "vendor" # one of "vendor", "librigidbodytracker"
            # only if enabled; see motion_capture.yaml
            marker: mocap_deck
            dynamics: default
            big_quad: false
            battery:
            voltage_warning: 3.8  # V
            voltage_critical: 3.7 # V

    # Keep default stuff here...

    ```

    In ```` motion_capture.yaml ````:

    ````yaml
    /motion_capture_tracking:
        ros__parameters:
            # one of "optitrack", "optitrack_closed_source", "vicon", "qualisys", "nokov", "vrpn", "motionanalysis"
            type: "qualisys"
            # Specify the hostname or IP of the computer running the motion capture software
            hostname: "192.168.1.140"

            topics:
            frame_id: "mocap"
            poses:
                qos:
                mode: "sensor"
                deadline: 100.0 # Hz
            tf:
                child_frame_id: "{}_mocap"
    
    # keep defaults .... 
    ````

    In motion_capture_tracking package, in ```cfg.yaml```:

    ```yaml
    /motion_capture_tracking:
        ros__parameters:
            # one of "optitrack", "optitrack_closed_source", "vicon", "qualisys", "nokov", "vrpn", "motionanalysis"
            type: "qualisys"
            # Specify the hostname or IP of the computer running the motion capture software
            hostname: "192.168.1.140"

    # Keep default stuff here...

    ```

- Wait for the EKF on drone to converge before launching any scrips to be safe, 15 secodns should be enough (EKF most likley uses the position data from QTM, droen does not take pose data directly to ensure it works smooth in case of dropouts, just my guess).

- Once it is running optimally, the topic list seen when running cflib backend was:

    ```bash
    /cf231/cmd_full_state
    /cf231/cmd_hover
    /cf231/cmd_position
    /cf231/cmd_vel_legacy
    /cf231/cmd_velocity_world
    /cf231/pose               # <--- The pose of our single drone
    /cf231/robot_description
    /cf231/status
    /cmd_full_state
    /cmd_vel
    /cmd_vel_legacy
    /joy
    /joy/set_feedback
    /parameter_events
    /pointCloud
    /poses                    # <--- The poses of all connected drones
    /rosout
    /tf
    /tf_static
    ```


### Tools to debug / confirm connection and packets recived from QTM

Install tools:
```bash
apt-get update
apt-get install -y iputils-ping tcpdump
```

If the following command does not work, there is a network issue between the container shell and the QTM pc (if no response, debug network connection between the 2 machiens and also check network settigns in docker-compose.yml):
```bash
ping <QTM_COMPUTER_IP>
```

To check weather motion_capture_tracking in ros recives UDP packets from QTM or not, run this:
```bash
 tcpdump -i any udp port 6734
```
 This will show a lot of packets beign recived once QTM is streaming and the cflib backend is running AND is connected to the drone without failure.


To confirm the MOCAP data from QTM is published (and to check that the position data makes sense) echo these topics (/poses will show poses for all avaible robots):

```bash
ros2 topic echo /poses
```


## Test flight

Could look like this for doing mroe exciting flight:
```bash
ros2 service call /cf231/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.5, duration: {sec: 2, nanosec: 0}, group_mask: 0}"
```