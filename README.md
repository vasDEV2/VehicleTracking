# Stalker v4.7

## Overview

Stalker Drone v4.7 is a Python-based drone tracking system that utilizes ArUco marker detection and tracking to follow a target. The system integrates with MAVLink-based drones using DroneKit and provides autonomous flight and target tracking capabilities.

## Features

- **Autonomous Takeoff & Navigation**: The drone arms, takes off, and navigates to a predefined location.
- **ArUco Marker Detection**: Uses OpenCV to detect and track ArUco markers.
- **Object Tracking**: Employs KCF tracking to follow a detected marker.
- **Gimbal Control**: Adjusts the gimbal to maintain focus on the target.
- **Waypoint Navigation**: If tracking is lost, follows a predefined search pattern.
- **MAVLink Communication**: Uses pymavlink to send velocity and navigation commands.


## DEMO
This is a clip of one of the low-speed flight test at the DTU sports ground. 
![Demo](DEMO/testing_dtu.gif)


## Dependencies

Ensure you have the following dependencies installed before running the script:

```bash
pip install opencv-python numpy dronekit pymavlink
```

## Hardware Requirements

- A MAVLink-compatible drone (e.g., ArduCopter-based drones)
- A gimbal-mounted camera
- A companion computer running Python

## Usage

### 1. Connect the Drone

Ensure your drone is connected and running MAVProxy or similar software. The script connects to the drone using:

```python
vehicle = connect("127.0.0.1:14555", wait_ready=False)
```

Modify the connection string if needed for your setup.

### 2. Run the Script

Execute the script with:

```bash
python stalker_v4.7.py
```

### 3. ArUco Marker Detection

Place an ArUco marker in the drone's field of view. The drone will:

1. Detect the marker.
2. Lock onto it using a tracker.
3. Adjust its gimbal to maintain focus.
4. Move towards the target if it drifts beyond a defined range.

### 4. Lost Tracking Handling

- If the marker is lost, the drone follows a predefined search pattern.
- It reattempts to lock onto the marker.

## Notes

- Modify the hardcoded waypoints (`lats` and `lons`) for your area.
- Ensure the camera is functional before running the script.


## Authors

Developed by V R Vasudevan.


