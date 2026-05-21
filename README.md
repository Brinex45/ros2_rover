# IRC 2026 — ROS 2 Rover (Kartikeya)

**ROS 2 Jazzy rover stack built for the International Rover Challenge 2026 by Team Automatons.**

The rover — named *Kartikeya* — is a 6-wheeled differential drive platform with an integrated robotic arm, GPS, encoder feedback, real-time telemetry, and live video streaming. It communicates wirelessly over WiFi at distances of **500+ metres** using a tuned FastDDS/FastRTPS middleware profile.

---

## Features

- ROS 2 Jazzy architecture with modular package separation
- `ros2_control` hardware interfaces for drive base and robotic arm
- micro-ROS bridge for embedded controller communication
- 6-wheel differential drive with encoder feedback
- 5-DOF robotic arm with inverse kinematics node
- PS4 controller support (drive + arm on separate controllers)
- Web-based interface — control the rover from any browser, no ROS install needed
- Long-range WiFi communication (500 m+) with FastDDS/FastRTPS tuning
- Live video streaming pipeline
- GPS integration (u-blox)
- RViz2 visualisation with custom URDF/Xacro robot description

---

## Repository Structure

```
src/
├── irc_rover_bringup        # Top-level launch files and controller config
├── irc_rover_description    # Full rover URDF (chassis + arm), meshes
├── rover_base_bringup       # Rover-only launch and joystick config
├── rover_base_description   # Rover chassis URDF and twist_to_stamped node
├── rover_base_hardware      # ros2_control SystemInterface for drive base
├── arm_bringup              # Arm launch files and controller config
├── arm_description          # Arm URDF, meshes, inverse kinematics node
├── arm_hardware             # ros2_control SystemInterface for arm joints
├── irc_interfaces           # Custom ROS 2 message definitions (Ps4, ArmAngles)
├── ps4                      # PS4 controller driver node (pygame-based)
├── ublox                    # u-blox GPS metapackage
├── ublox_gps                # u-blox GPS driver node
├── ublox_msgs               # u-blox message definitions
└── ublox_serialization      # u-blox serialization utilities
```

---

## System Architecture

```
PS4 Controller / Web Interface
           │
    ROS 2 Topics (WiFi / FastDDS)
           │
    Control Nodes
    (ps4_data_node, ps4_data_to_twist, arm_kinematics_node)
           │
    ros2_control Controller Manager
    (rover_base_controller, arm_joints_controller)
           │
    Hardware Interface (SystemInterface)
    (rover_base_hardware, arm_hardware)
           │
    micro-ROS Agent (serial)
           │
    Embedded Controller
           │
    Motors / Encoders / Sensors
```

---

## Communication System

The rover communicates over WiFi using **FastDDS (FastRTPS)** as the ROS 2 middleware. A custom XML profile (`fastrtps.xml`) is included in the repository root and is **critical** for stable long-range wireless operation.

The profile:
- Restricts DDS discovery to the selected network interface
- Reduces unnecessary multicast traffic over the wireless link
- Optimises UDP transport for long-range WiFi
- Enables reliable topic communication at 500 m+

**Set the profile before launching any node:**

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastrtps.xml
```

---

## Hardware Stack

| Category  | Components                                    |
|-----------|-----------------------------------------------|
| Compute   | Ubuntu 24.04, ROS 2 Jazzy, micro-ROS          |
| Drive     | 6-wheel differential drive, wheel encoders    |
| Arm       | 5-DOF robotic arm, custom IK node             |
| GPS       | u-blox module (ublox_gps driver)              |
| Control   | PS4 controller (wired/wireless via pygame)    |
| Comms     | Long-range WiFi, FastDDS, video stream        |

---

## Web Interface

The rover supports a browser-based control interface that communicates with ROS 2 topics over the network. This allows:

- Rover driving and arm control from any device
- Live telemetry visualisation
- Video stream viewing
- ROS topic monitoring

No ROS installation is required on the client device.

---

## Build and Launch

### Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- micro-ROS agent
- `ros2_control`, `controller_manager`
- `pygame` (for PS4 node)

### Clone and Build

```bash
git clone https://github.com/Brinex45/ros2_rover.git
cd ros2_rover
colcon build --symlink-install
source install/setup.bash
```

### Set FastDDS Profile

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastrtps.xml
```

### Launch

**Full rover + arm:**
```bash
ros2 launch irc_rover_bringup irc_rover.launch.xml
```

**Drive base only:**
```bash
ros2 launch rover_base_bringup rover_base.launch.py
```

**Arm only:**
```bash
ros2 launch arm_bringup arm.launch.py
```

**micro-ROS agent (connect embedded controller):**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

**RViz visualisation:**
```bash
rviz2
```

---

## PS4 Controller

Two PS4 controllers are supported simultaneously — one for rover drive control and one for arm control. The `ps4_data_node` detects connected joysticks automatically and publishes on separate topics:

- `/ps4_data_rover` — drive commands
- `/ps4_data_arm` — arm joint commands

The Rover's PS4 has more priority i.e. if the Rover PS4 gets disconnected the Arm's PS4 is shifted to Rover so that the Navigation is not hindered
---

## ros2_control Packages

| Package              | Interface Type  | Responsibility             |
|----------------------|-----------------|----------------------------|
| `rover_base_hardware`| SystemInterface | Wheel velocity + encoder feedback |
| `arm_hardware`       | SystemInterface | Arm joint position control |

Both packages communicate with the embedded controller via micro-ROS topics over serial.

---

## Custom Messages (`irc_interfaces`)

| Message      | Fields                        | Used for            |
|--------------|-------------------------------|---------------------|
| `Ps4`        | `ps4_data_analog`, `ps4_data_buttons` | Controller input    |
| `ArmAngles`  | Joint angle array             | Arm IK output       |

---

## Future Goals

- Nav2 integration
- Sensor fusion with EKF
- SLAM integration
- Autonomous waypoint navigation
- Visual-inertial odometry
- Recovery behaviours
- Mission planner

---

## Technologies Used

| | |
|---|---|
| ROS 2 Jazzy | FastDDS / FastRTPS |
| micro-ROS | ros2_control |
| RViz2 | Xacro / URDF |
| C++ | Python |
| Ubuntu 24.04 | u-blox GPS |
| pygame | |

---

## Team

Developed for the **International Rover Challenge 2026** by **Team Automatons**.

---

> The rover software stack is actively evolving. New capabilities in localization, communication, and control are continuously being added.
