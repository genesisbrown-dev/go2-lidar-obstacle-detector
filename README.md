# go2-lidar-obstacle-detector

> **ROS2 Humble · NVIDIA Isaac Sim · Unitree Go2**  
> Real-time LiDAR obstacle classification for the Unitree Go2 quadruped robot.  
> Developed as part of a Computer Engineering robotics research project.

---

## Table of Contents

1. [Project Overview](#project-overview)  
2. [Architecture](#architecture)  
3. [Safety States](#safety-states)  
4. [Folder Structure](#folder-structure)  
5. [Prerequisites](#prerequisites)  
6. [Installation](#installation)  
7. [Running in Isaac Sim](#running-in-isaac-sim)  
8. [Running on the Physical Go2](#running-on-the-physical-go2)  
9. [Testing](#testing)  
10. [Configuration](#configuration)  
11. [Troubleshooting](#troubleshooting)  
12. [Roadmap](#roadmap)  
13. [License](#license)  

---

## Project Overview

`go2-lidar-obstacle-detector` is a ROS2 Humble Python node that:

- Subscribes to `/unitree_go2_0/lidar/point_cloud` (PointCloud2)
- Finds the **nearest obstacle** in the 3-D point cloud using Euclidean distance
- Prints one of three **safety states** to the console in real time

The project is designed **simulation-first**: develop and validate every feature in NVIDIA Isaac Sim before deploying to the physical Unitree Go2 hardware. Because Isaac Sim publishes the same ROS2 topic and message types as the real robot driver, **zero code changes are required** when switching from simulation to hardware.

---

## Architecture

```
NVIDIA Isaac Sim  ──or──  Unitree Go2 hardware
        │                          │
        │  PointCloud2             │  PointCloud2
        └──────────────┬───────────┘
                       ▼
          /unitree_go2_0/lidar/point_cloud
                       │
                       ▼
          ┌────────────────────────┐
          │   LidarDetectorNode    │
          │  (lidar_detector.py)   │
          │                        │
          │  1. Iterate all points │
          │  2. Compute min dist   │
          │  3. Classify state     │
          └────────────┬───────────┘
                       │
              Console output
           ✅ SAFE / ⚠️ CAUTION / 🛑 STOP
```

---

## Safety States

| State       | Condition               | Meaning                          |
|-------------|-------------------------|----------------------------------|
| ✅ SAFE    | distance > 2.0 m        | Path is clear, robot may proceed |
| ⚠️ CAUTION | 1.0 m ≤ distance ≤ 2.0 m | Slow down, monitor surroundings  |
| 🛑 STOP    | distance < 1.0 m        | Halt immediately                 |

Thresholds are fully configurable via `config/detector_params.yaml`.

---

## Folder Structure

```
go2-lidar-obstacle-detector/
│
├── go2_lidar_detector/          # Python package (ROS2 node source)
│   ├── __init__.py
│   └── lidar_detector.py        # ← Main node: subscriber + classifier
│
├── launch/
│   └── lidar_detector.launch.py # ROS2 launch file
│
├── config/
│   └── detector_params.yaml     # Tunable distance thresholds
│
├── tests/
│   └── test_classifier.py       # Pytest unit tests (no ROS2 needed)
│
├── resource/
│   └── go2_lidar_detector       # ament_index marker (required by ROS2)
│
├── package.xml                  # ROS2 package manifest & dependencies
├── setup.py                     # ament_python build configuration
├── setup.cfg                    # ament_python install paths
├── requirements.txt             # Pure-Python dev/test dependencies
└── README.md
```

---

## Prerequisites

### System

| Requirement        | Version / Notes                          |
|--------------------|------------------------------------------|
| Ubuntu             | 22.04 LTS (Jammy)                        |
| ROS2               | Humble Hawksbill                         |
| Python             | 3.10+                                    |
| NVIDIA Isaac Sim   | 4.x (for simulation)                     |
| Unitree Go2 driver | `unitree_ros2` (for physical robot)      |

### ROS2 packages

```bash
sudo apt install -y \
  ros-humble-sensor-msgs \
  python3-sensor-msgs-py
```

---

## Installation

### 1. Create (or use an existing) ROS2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone this repository

```bash
git clone https://github.com/<your-username>/go2-lidar-obstacle-detector.git
```

### 3. Install Python dev dependencies

```bash
cd go2-lidar-obstacle-detector
pip install -r requirements.txt
```

### 4. Resolve ROS2 dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build the workspace

```bash
colcon build --symlink-install --packages-select go2_lidar_detector
```

The `--symlink-install` flag means you can edit `lidar_detector.py` and
re-run without rebuilding.

### 6. Source the workspace

```bash
source install/setup.bash

# Add to ~/.bashrc to avoid typing this every session:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running in Isaac Sim

Isaac Sim is the recommended way to develop and validate the node before
touching the physical robot.

### Step 1 – Launch Isaac Sim with the Go2 scene

1. Open **NVIDIA Isaac Sim**.
2. Load the Unitree Go2 USD asset or use the provided Go2 sample scene.
3. Enable the **ROS2 Bridge** extension:  
   `Window → Extensions → search "ROS2 Bridge" → Enable`
4. Press **Play** (▶) to start the simulation.

Isaac Sim will begin publishing:
```
/unitree_go2_0/lidar/point_cloud   (sensor_msgs/PointCloud2)
```

Verify the topic is live:
```bash
ros2 topic list | grep point_cloud
ros2 topic hz /unitree_go2_0/lidar/point_cloud
```

### Step 2 – Run the detector node

**Option A – via launch file (recommended):**
```bash
ros2 launch go2_lidar_detector lidar_detector.launch.py
```

**Option B – direct run:**
```bash
ros2 run go2_lidar_detector lidar_detector
```

**Option C – override the topic at launch time:**
```bash
ros2 launch go2_lidar_detector lidar_detector.launch.py \
    lidar_topic:=/your/custom/topic
```

### Step 3 – Observe output

Move objects in the Isaac Sim scene toward the robot to trigger state changes:

```
[msg #1]  Nearest obstacle: 3.412 m  →  ✅  SAFE
[msg #2]  Nearest obstacle: 1.873 m  →  ⚠️  CAUTION
[msg #3]  Nearest obstacle: 0.654 m  →  🛑  STOP
```

---

## Running on the Physical Go2

> ⚠️ **Safety first.** Test on the physical robot only after validating in simulation.  
> Keep the emergency stop button within reach at all times.

### Step 1 – Set up the Unitree ROS2 driver

Follow the [unitree_ros2 installation guide](https://github.com/unitreerobotics/unitree_ros2)
to connect your PC to the Go2 over Ethernet or Wi-Fi and start the driver.

The driver publishes on the same topic:
```
/unitree_go2_0/lidar/point_cloud
```

### Step 2 – Run the detector

```bash
# No code changes needed — same command as simulation
ros2 launch go2_lidar_detector lidar_detector.launch.py
```

### Step 3 – Validate

Walk toward the robot and confirm SAFE → CAUTION → STOP transitions appear
in the terminal as expected.

---

## Testing

Unit tests cover the safety classifier logic without requiring ROS2 or Isaac Sim.

### Run all tests

```bash
# From the project root
pytest tests/ -v
```

### Expected output

```
tests/test_classifier.py::TestClassify::test_safe_far          PASSED
tests/test_classifier.py::TestClassify::test_safe_boundary     PASSED
tests/test_classifier.py::TestClassify::test_caution_middle    PASSED
tests/test_classifier.py::TestClassify::test_caution_lower_boundary  PASSED
tests/test_classifier.py::TestClassify::test_caution_upper_boundary  PASSED
tests/test_classifier.py::TestClassify::test_stop_close        PASSED
tests/test_classifier.py::TestClassify::test_stop_boundary     PASSED
tests/test_classifier.py::TestClassify::test_stop_zero         PASSED

8 passed in 0.XX s
```

### Run with coverage

```bash
pytest tests/ --cov=go2_lidar_detector --cov-report=term-missing
```

### Run ROS2 colcon tests

```bash
colcon test --packages-select go2_lidar_detector
colcon test-result --verbose
```

---

## Configuration

Edit `config/detector_params.yaml` to tune thresholds without touching code:

```yaml
lidar_detector_node:
  ros__parameters:
    lidar_topic:      "/unitree_go2_0/lidar/point_cloud"
    safe_distance:    2.0   # metres – above this → SAFE
    caution_distance: 1.0   # metres – above this → CAUTION; below → STOP
```

Reload at runtime:
```bash
ros2 param load /lidar_detector_node config/detector_params.yaml
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `No module named 'sensor_msgs_py'` | Package not installed | `sudo apt install python3-sensor-msgs-py` |
| `Warning: Point cloud is empty` | Isaac Sim not publishing / scene not playing | Press **Play** in Isaac Sim; check ROS2 Bridge is enabled |
| `ros2 topic list` shows no LiDAR topic | ROS2 Bridge not enabled or wrong DDS | Enable bridge extension; check `ROS_DOMAIN_ID` |
| Node exits immediately | Missing `source install/setup.bash` | Source the workspace overlay |
| All readings show SAFE | LiDAR z-axis pointing up, no floor hit | Rotate sensor in scene or filter `z < threshold` |

---

## Roadmap

- [ ] Read thresholds from `detector_params.yaml` at runtime (ROS2 params)  
- [ ] Publish `/obstacle_state` as a `std_msgs/String` topic  
- [ ] Add angular sector filtering (front-only, 180°, etc.)  
- [ ] Integrate velocity commands to automatically slow/stop the Go2  
- [ ] Add RViz2 marker publisher to visualise the nearest point  
- [ ] GitHub Actions CI for automated `pytest` on every push  

---

## License

MIT License — see [LICENSE](LICENSE) for details.

---

*Built with ROS2 Humble · NVIDIA Isaac Sim · Unitree Go2*
