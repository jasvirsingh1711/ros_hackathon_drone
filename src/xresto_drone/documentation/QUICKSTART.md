# Quick Start Guide

## Prerequisites

Ensure you have ROS 2 (Humble recommended) and Gazebo 11+ installed.

```bash
# Verify installation
ros2 --version  # Should be ROS 2 Humble (or newer)
gazebo --version  # Should be 11+
which colcon  # Should return path to colcon
```

---

## Step 1: Build the Package

```bash
cd ~/xresto_ws
colcon build --symlink-install --packages-select xresto_drone
source install/setup.bash
```

**Expected Output:**
```
Compiling xresto_drone
[100%] Linking CXX gate_detector executable...
[100%] Built target xresto_drone
Summary: 1 package finished successfully
```

**Troubleshooting:**
- If you see "Missing package", run: `rosdep install --from-paths src --ignore-src -r -y`
- If CMake errors occur, try: `colcon build --packages-select xresto_drone --cmake-args -DCMAKE_BUILD_TYPE=Release`

---

## Step 2: Launch the System

**Full Autonomous Mission (Recommended):**
```bash
ros2 launch xresto_drone autonomous_mission.launch.py
```

This starts the complete system:
- Gazebo simulator with world
- Drone spawning at origin
- All autonomous modules (perception, navigation, planning, control)
- Optional RViz visualization

**Startup Timeline:**
- 0-5s: Gazebo loads
- 5-8s: Drone appears in simulation
- 8-10s: Nodes initialize
- 10s+: Autonomous mission begins

---

## Step 3: Monitor the System

**Watch Sensor Topics:**
```bash
# Gate detections from perception layer
ros2 topic echo /drone/detected_gates

# Navigation state (SEARCH/APPROACH/TRAVERSE)
ros2 topic echo /drone/nav_state

# Planned trajectory waypoints
ros2 topic echo /drone/planned_trajectory

# Drone status and diagnostics
ros2 topic echo /drone/status
```

**List All Topics:**
```bash
ros2 topic list
```

**View Topic Frequency & Data Rate:**
```bash
ros2 topic hz /drone/detected_gates  # Should be ~30 Hz
ros2 topic bw /drone/front_camera/image_raw  # Monitor bandwidth
```

---

## Step 4: Run Individual Components (for Testing)

**Terminal 1: Gazebo & ROS Bridge**
```bash
ros2 launch xresto_drone spawn_drone.launch.py
```

**Terminal 2: Perception (Gate Detection)**
```bash
ros2 run xresto_drone gate_detector
# Reads: /drone/front_camera/image_raw, /drone/front_camera/depth/image_raw
# Publishes: /drone/detected_gates
```

**Terminal 3: Navigation (State Machine)**
```bash
ros2 run xresto_drone gate_navigator.py
# Reads: /drone/detected_gates
# Publishes: /drone/nav_state, /drone/target_gate
```

**Terminal 4: Planning (Trajectory Generation)**
```bash
ros2 run xresto_drone trajectory_planner.py
# Reads: /drone/nav_state, /drone/target_gate
# Publishes: /drone/planned_trajectory, /drone/cmd_force
```

**Terminal 5: Control (PID Stabilization)**
```bash
ros2 run xresto_drone hover_node.py
# Reads: /drone/cmd_force, /drone/imu, /drone/altitude
# Publishes: Motor thrust commands to Gazebo
```

---

## Understanding the Data Flow

```
Gate Detection (Perception)
        ↓ Gate positions + confidence
Navigation (Sequencing)
        ↓ Navigation state + target gate
Trajectory Planning (Path Generation)
        ↓ Force commands + waypoints
Hover Control (PID Stabilization)
        ↓ Motor thrust commands
Gazebo Physics
        ↓ Drone motion & sensor updates
        ↑ (Loop closes with sensor feedback)
```

---

## Key ROS Topics Explained

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/drone/detected_gates` | Detection data | Perception → Nav | Gate positions and confidence scores |
| `/drone/nav_state` | String | Nav → Planner | Current state: SEARCH, APPROACH, TRAVERSE |
| `/drone/target_gate` | Data | Nav → Planner | Which gate to target next |
| `/drone/planned_trajectory` | Waypoints | Planner → Control | Path to follow |
| `/drone/cmd_force` | Wrench/Force | Planner → Control | Desired force/acceleration commands |
| `/drone/imu` | IMU data | Gazebo → Control | Acceleration and angular rates (100 Hz) |
| `/drone/altitude` | Range | Gazebo → Control | Height above ground (40 Hz) |
| `/drone/odometry` | Odometry | Gazebo → Navigation | Position and velocity (50 Hz) |

---

## Expected Performance

| Metric | Target | Typical |
|--------|--------|---------|
| Perception rate (gate detection) | 30 Hz | 30 Hz |
| Navigation updates | 10 Hz | 10 Hz |
| Control loop rate | 50-100 Hz | 100 Hz |
| Altitude accuracy | Within 0.2m | ±0.1m |
| Attitude stability | <5° oscillation | 2-4° |
| Mission autonomy | 100% | 100% (zero manual input) |

---

## Debugging Tips

**Drone not spawning:**
- Check Gazebo is fully loaded (5+ seconds wait)
- Verify `/clock` topic is publishing: `ros2 topic echo /clock`

**No gate detections:**
- Check camera images: `ros2 run rqt_image_view`
- Verify HSV thresholds in `config/drone_params.yaml`
- Check gate colors match expected HSV ranges

**Drone drifting or unstable:**
- Review PID gains in `config/drone_params.yaml`
- Check IMU is providing data: `ros2 topic echo /drone/imu`
- Verify LiDAR altitude readings: `ros2 topic echo /drone/altitude`

**Mission not starting:**
- Ensure all nodes started: `ros2 node list`
- Check for errors: `ros2 node info /gate_detector`
- Monitor stdout: `ros2 launch xresto_drone autonomous_mission.launch.py 2>&1 | grep ERROR`

---

## Resources

- **ROS 2 Official:** docs.ros.org
- **Gazebo Simulation:** gazebosim.org
- **System Configuration:** `config/drone_params.yaml`
- **Technical Details:** `TECHNICAL_REPORT.md`
