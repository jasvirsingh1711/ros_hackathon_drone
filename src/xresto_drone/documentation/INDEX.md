# XRESTO Drone Autonomous Gate Navigation - Submission Package
## Complete File Index and Navigation Guide

**Project:** IIT Mandi Tech Fest 2026 - Autonomous Drone Gate Navigation  
**Team:** XRESTO  
**Status:** Final Submission Ready  
**Date:** March 2026

---

## 📋 Quick Navigation

### For First-Time Users
1. Start with: [README.md](README.md) - System overview
2. Then read: [QUICKSTART.md](QUICKSTART.md) - Installation and launch
3. Watch: [DEMO_VIDEO_README.md](DEMO_VIDEO_README.md) - Record/understand demo video

### For Technical Review
1. [TECHNICAL_REPORT.md](TECHNICAL_REPORT.md) - Complete system documentation
2. [IMPLEMENTATION.md](IMPLEMENTATION.md) - Detailed architecture
3. [SUBMISSION_CHECKLIST.md](SUBMISSION_CHECKLIST.md) - Deliverables verification

### For Development & Maintenance
1. [config/drone_params.yaml](config/drone_params.yaml) - System tuning parameters
2. [src/](src/) - Source code modules
3. [TECHNICAL_REPORT.md#section-5-control-logic](TECHNICAL_REPORT.md) - PID tuning guide

### For Simulation Setup
1. [worlds/gate_course.world](worlds/gate_course.world) - 4-gate environment
2. [urdf/Final_Drone.urdf](urdf/Final_Drone.urdf) - Drone model with sensors
3. [meshes/](meshes/) - 3D mesh assets

### For Visualization
1. [rviz/drone_viz.rviz](rviz/drone_viz.rviz) - Main visualization config
2. [docs/RVIZ_CONFIGURATION.md](docs/RVIZ_CONFIGURATION.md) - Usage guide

---

## 📁 Directory Structure & File Descriptions

### Root Configuration Files

**Documentation & Meta:**
| File | Purpose | Audience |
|------|---------|----------|
| [README.md](README.md) | Project overview, system architecture, features | Everyone |
| [QUICKSTART.md](QUICKSTART.md) | Installation steps, build instructions, launch | New users |
| [TECHNICAL_REPORT.md](TECHNICAL_REPORT.md) | Complete technical documentation (11 sections) | Technical reviewers |
| [IMPLEMENTATION.md](IMPLEMENTATION.md) | Detailed architecture and module descriptions | Developers |
| [SUBMISSION_CHECKLIST.md](SUBMISSION_CHECKLIST.md) | Deliverables verification checklist | Evaluators |
| [DEMO_VIDEO_README.md](DEMO_VIDEO_README.md) | Demo video recording instructions & guide | Video creators |
| [INDEX.md](INDEX.md) | This file - navigation guide | Everyone |

**Build Configuration:**
| File | Purpose |
|------|---------|
| [CMakeLists.txt](CMakeLists.txt) | ROS 2 build configuration for C++ modules |
| [package.xml](package.xml) | ROS package definition and dependencies |

---

### 🎯 config/ - System Parameters

| File | Purpose | Contains |
|------|---------|----------|
| [drone_params.yaml](config/drone_params.yaml) | All tunable system parameters | PID gains, velocity limits, detection thresholds |

**Parameter Categories:**
- Altitude controller: Kp=3.0, Ki=0.5, Kd=2.0
- Attitude controller: Kp=4.0, Ki=0.5, Kd=1.2
- Velocity limits: 1.5 m/s XY, 1.0 m/s Z
- Gate detection: HSV thresholds, confidence threshold
- Safety limits: Collision distance, timeouts

---

### 🚀 launch/ - Launch Files

| File | Purpose | Usage |
|------|---------|-------|
| [autonomous_mission.launch.py](launch/autonomous_mission.launch.py) | **Main launcher** - Complete mission | `ros2 launch xresto_drone autonomous_mission.launch.py` |
| [spawn_drone.launch.py](launch/spawn_drone.launch.py) | Spawn drone only (for development) | Debugging individual components |
| [spawn_gate_world.launch.py](launch/spawn_gate_world.launch.py) | Load gate course world | Gazebo environment setup |
| [display.launch.py](launch/display.launch.py) | Launch visualization (Gazebo + RViz) | Visual debugging |

**Recommended:** Use `autonomous_mission.launch.py` for all standard runs.

---

### 💾 src/ - Source Code Modules

**Core Autonomous Control Modules:**

| File | Language | Lines | Purpose |
|------|----------|-------|---------|
| [gate_detector.cpp](src/gate_detector.cpp) | C++ | ~500 | **Perception:** Real-time gate detection via HSV color segmentation |
| [gate_navigator.py](src/gate_navigator.py) | Python | ~300 | **Navigation:** Gate sequencing, state machine FSM |
| [trajectory_planner.py](src/trajectory_planner.py) | Python | ~350 | **Planning:** Path generation, waypoint calculation |
| [hover_node.py](src/hover_node.py) | Python | ~400 | **Control:** Cascaded PID controllers, motor commands |

**Module Responsibilities:**
1. **gate_detector.cpp** → RGB/Depth frames → Gate positions + confidence
2. **gate_navigator.py** → Gate positions → Navigation state + target
3. **trajectory_planner.py** → Navigation state → Waypoints + force commands
4. **hover_node.py** → Force commands + sensor feedback → Motor thrusts

---

### 🐚 urdf/ - Robot Descriptions and Models

| File | Type | Purpose | Contains |
|------|------|---------|----------|
| [Final_Drone.urdf](urdf/Final_Drone.urdf) | URDF | **Main drone model** | Quadrotor structure with sensors (camera, depth, IMU, LiDAR) |
| [red_gate.urdf](urdf/red_gate.urdf) | URDF | Gate obstacle model | Geometric definition of red X-frame gate |

**Final_Drone.urdf Specifications:**
- Mass: 0.758 kg
- Propeller configuration: X-frame (4 motors)
- Sensors: RGB camera, depth camera, IMU, LiDAR (altitude)
- Motor specs: 0-10 N thrust each
- Physics: Realistic ODE physics engine

---

### 🌍 worlds/ - Gazebo Simulation Environments

| File | Purpose | Gates | Notes |
|------|---------|-------|-------|
| [gate_course.world](worlds/gate_course.world) | **Primary course** | 4 gates | Standard competition environment |
| [aerial_nav.world](worlds/aerial_nav.world) | Alternative environment | Other configs | Backup world file |

**Gate Course Layout:**
```
Gate 0 (Red):   (0, 0, 0)    0° rotation     - Starting point
Gate 1 (Green): (5, 0, 0)    90° rotation    - Orientation change
Gate 2 (Blue):  (8, 3, 0)    11.5° tilt      - Tilted frame
Gate 3 (Yellow):(10, 5, 0)   90°+tilt        - Complex maneuver
```

---

### 📦 meshes/ - 3D Model Assets

| File | Type | Purpose |
|------|------|---------|
| [base_link.STL](meshes/base_link.STL) | Mesh | Drone body/fuselage |
| [camera_link.STL](meshes/camera_link.STL) | Mesh | Camera module assembly |
| [prop1.STL](meshes/prop1.STL) | Mesh | Front-left propeller |
| [prop2.STL](meshes/prop2.STL) | Mesh | Front-right propeller |
| [prop3.STL](meshes/prop3.STL) | Mesh | Back-right propeller |
| [prop4.STL](meshes/prop4.STL) | Mesh | Back-left propeller |

**Notes:**
- All in STL format (standard for Gazebo)
- Units: meters
- Located at: `/meshes/` relative to URDF

---

### 🎨 rviz/ - Visualization Configurations

| File | Purpose | Displays |
|------|---------|----------|
| [drone_viz.rviz](rviz/drone_viz.rviz) | **Main RViz config** | Drone model, gates, trajectory, sensors |

**Displays Included:**
- Robot model (TF tree visualization)
- Gate positions (markers)
- Planned trajectory (path visualization)
- Camera frustums (sensor view)
- Point clouds (if enabled)
- Altitude sensor (LiDAR)

---

### 📚 docs/ - Additional Documentation

| File | Purpose | Covers |
|------|---------|--------|
| [ALTITUDE_VERIFICATION.md](docs/ALTITUDE_VERIFICATION.md) | LiDAR sensor verification | Mounting, orientation, calibration |
| [RVIZ_CONFIGURATION.md](docs/RVIZ_CONFIGURATION.md) | RViz setup guide | Display configuration, marker setup |

---

### 📁 include/ - C++ Headers (Build Support)

Contains C++ header files for `gate_detector` compilation.

---

### 🔄 models/ - Supporting Model Files

Contains SDF model definitions for simulation (Gazebo support files).

---

## 🎯 File Dependencies & Data Flow

```
ROSMaster
    ├─→ [gate_detector.cpp]
    │   Input:  /camera/rgb/image_raw, /camera/depth/image_raw, /imu/data
    │   Output: /gate_detection/pose, /gate_detection/state
    │
    ├─→ [gate_navigator.py]
    │   Input:  /gate_detection/pose, timer (10 Hz)
    │   Output: /navigation/state, /navigation/target_gate
    │
    ├─→ [trajectory_planner.py]
    │   Input:  /navigation/target_gate, /drone/odometry
    │   Output: /planner/waypoints, /planner/force_cmd
    │
    └─→ [hover_node.py]
        Input:  /planner/force_cmd, /imu/data, /lidar/range
        Output: /cmd_vel, /gazebo/apply_force

Gate Detection (HSV)
    ↓
Navigation FSM (State Machine)
    ↓
Trajectory Planning (Waypoints)
    ↓
PID Control (Motor Commands)
    ↓
Gazebo Physics
    ↓
Sensor Feedback
```

---

## 🚀 Quick Command Reference

### Installation
```bash
cd ~/xresto_ws
colcon build --packages-select xresto_drone
source install/setup.bash
```

### Main Launch
```bash
ros2 launch xresto_drone autonomous_mission.launch.py
```

### Build Only (No catkin)
```bash
colcon build --packages-select xresto_drone --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Check Compilation
```bash
cd build/xresto_drone
make
```

### View Topics
```bash
ros2 topic list
ros2 topic echo /gate_detection/pose
```

### Visualize with RViz
```bash
rviz2 -d rviz/drone_viz.rviz
```

### View Parameters
```bash
ros2 param get /hover_controller altitude_kp
```

---

## ✅ Pre-Launch Checklist

- [ ] ROS 2 installed and working
- [ ] Gazebo installed (version 11+)
- [ ] Package built successfully: `colcon build`
- [ ] Environment sourced: `source install/setup.bash`
- [ ] Sufficient free disk space (~2 GB)
- [ ] System has 8GB+ RAM
- [ ] No other Gazebo/RViz instances running
- [ ] GPU available (optional but recommended)

---

## 📊 System Specifications Summary

**Hardware (Simulated Drone):**
- Type: Quadrotor (X-frame)
- Mass: 0.758 kg
- Motors: 4× (0-10 N each)

**Sensors:**
- RGB Camera: 640×480@30Hz
- Depth Camera: Synchronized, 20m range
- IMU: 3-axis accel, 3-axis gyro, 100Hz
- LiDAR: Single-beam altitude, 40Hz

**Control Rates:**
- Altitude: 50 Hz
- Attitude: 100 Hz
- Navigation: 10 Hz
- Motor output: 100 Hz

**Performance Targets:**
- Gate detection: >95% accuracy
- Attitude stability: <5° oscillation
- Altitude accuracy: ±0.2 m
- Mission time: <120 seconds
- All 4 gates: 100% traverse success

---

## 🔍 Troubleshooting Quick Links

**Build Errors:**
→ See [QUICKSTART.md](QUICKSTART.md) - "Build Troubleshooting"

**Runtime Issues:**
→ See [TECHNICAL_REPORT.md#section-9](TECHNICAL_REPORT.md) - "Safety & Troubleshooting"

**Gate Detection Problems:**
→ See [config/drone_params.yaml](config/drone_params.yaml) - Adjust HSV thresholds

**Control Instability:**
→ See [TECHNICAL_REPORT.md#section-5.2](TECHNICAL_REPORT.md) - "PID Tuning Parameters"

---

## 📞 Support Resources

| Resource | Content |
|----------|---------|
| [TECHNICAL_REPORT.md](TECHNICAL_REPORT.md) | Complete system documentation |
| [IMPLEMENTATION.md](IMPLEMENTATION.md) | Detailed architecture specs |
| [config/drone_params.yaml](config/drone_params.yaml) | All tunable parameters |
| [docs/](docs/) | Additional technical guides |

---

## 📋 Submission Deliverables Checklist

Essential files for final submission:

✅ **Code & Build:**
- CMakeLists.txt
- package.xml
- src/ (all 4 modules)

✅ **Configuration:**
- config/drone_params.yaml
- launch/autonomous_mission.launch.py

✅ **Models & Assets:**
- urdf/Final_Drone.urdf
- meshes/ (6 STL files)
- worlds/gate_course.world

✅ **Documentation:**
- README.md
- QUICKSTART.md
- TECHNICAL_REPORT.md
- SUBMISSION_CHECKLIST.md
- DEMO_VIDEO_README.md

✅ **Visualization:**
- rviz/drone_viz.rviz

**Total Files:** ~25 files  
**Source Code:** ~1,600 lines  
**Documentation:** ~3,000 lines  

---

## 📝 Document History

| Date | Status | Notes |
|------|--------|-------|
| March 2026 | Final | Complete submission package ready |

---

## 🎓 Understanding the System

### For Quick Understanding:
Read in order: [README.md](README.md) → [QUICKSTART.md](QUICKSTART.md) → [Launch & Run]

### For Deep Technical Understanding:
Read: [TECHNICAL_REPORT.md](TECHNICAL_REPORT.md) → [IMPLEMENTATION.md](IMPLEMENTATION.md) → [Source Code](src/)

### For Modification/Tuning:
Edit: [config/drone_params.yaml](config/drone_params.yaml) and source files

### For Integration/Deployment:
Follow: [QUICKSTART.md](QUICKSTART.md) build steps and [launch/autonomous_mission.launch.py](launch/autonomous_mission.launch.py)

---

**Navigation Guide Prepared By:** XRESTO Team  
**Last Updated:** March 2026  
**Status:** Final Submission Ready
