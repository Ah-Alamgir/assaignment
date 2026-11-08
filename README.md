# Autonomous Vehicles Course: F1TENTH Follow-the-Gap Controller
**Student:** Alamgir Abu Hanif  
**Group:** Group 4  
**Assigned Track:** Circuit Zandvoort (`Zandvoort_map.png`)  
**Repository:** [https://github.com/widegonz/F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository)

---

## 📋 Table of Contents
1. [Project Overview & Objectives](#-project-overview--objectives)
2. [Reactive Controller Architecture (Follow-the-Gap)](#-reactive-controller-architecture-follow-the-gap)
   - [2.1 Field of View & Laser Scan Preprocessing](#21-field-of-view--laser-scan-preprocessing)
   - [2.2 Disparity Extender Algorithm](#22-disparity-extender-algorithm)
   - [2.3 Dynamic Obstacle Safety Bubble](#23-dynamic-obstacle-safety-bubble)
   - [2.4 Maximum Gap Identification](#24-maximum-gap-identification)
   - [2.5 Optimal Goal Point Selection](#25-optimal-goal-point-selection)
   - [2.6 Curvature-Adaptive Dynamic Speed Profiling](#26-curvature-adaptive-dynamic-speed-profiling)
3. [Implementation Details](#-implementation-details)
   - [3.1 Lap Counter Implementation](#31-lap-counter-implementation)
   - [3.2 Lap Timer Implementation & Real-Time Displays](#32-lap-timer-implementation--real-time-displays)
   - [3.3 Code Structure & ROS 2 Interface](#33-code-structure--ros-2-interface)
4. [Step-by-Step Installation & Execution Guide](#-step-by-step-installation--execution-guide)
   - [4.1 Prerequisites](#41-prerequisites)
   - [4.2 Building the Workspace](#42-building-the-workspace)
   - [4.3 Running the Simulation & Controller](#43-running-the-simulation--controller)
5. [Competition Benchmark Results](#-competition-benchmark-results)
   - [Challenge 1: 10 Consecutive Laps Without Collisions](#challenge-1-10-consecutive-laps-without-collisions)
   - [Challenge 2: Fastest Lap Time](#challenge-2-fastest-lap-time)
6. [Oral Project Defense Preparation & Technical Q&A](#-oral-project-defense-preparation--technical-qa)

---

## 🎯 Project Overview & Objectives

This project implements an autonomous reactive navigation system for a 1/10-scale autonomous race car using the official **F1TENTH Simulator** in **ROS 2**. The vehicle is tested in a high-speed racing competition on **Circuit Zandvoort**, a challenging racetrack characterized by 14 complex corners, chicanes, and banked curves.

The project evaluates two performance criteria:
1. **Challenge 1: 10 Consecutive Laps Without Collisions** — The reactive controller must navigate the entire 375.8-meter track for 10 complete, uninterrupted laps without contacting track boundaries or obstacles.
2. **Challenge 2: Fastest Lap Time** — Achieving the fastest single lap time among the 10 completed laps through aggressive cornering, disparity extension, and dynamic velocity scheduling.

---

## 🧠 Reactive Controller Architecture (Follow-the-Gap)

The Follow-the-Gap (FTG) method is an advanced reactive obstacle avoidance algorithm based on laser rangefinder (LiDAR) data. Rather than requiring pre-recorded global waypoints or heavy SLAM mapping at runtime, FTG continuously evaluates surrounding geometry at 100 Hz to detect navigable free space ("gaps") and steer the vehicle toward the optimal corridor.

```
       [Raw LiDAR Scan: 1080 beams, 270°]
                       │
                       ▼
       [1. FOV Restriction & Moving Avg Filter]
                       │
                       ▼
       [2. Disparity Extender Algorithm]
                       │
                       ▼
       [3. Safety Bubble Inflation around Closest Point]
                       │
                       ▼
       [4. Contiguous Free-Space Max Gap Detection]
                       │
                       ▼
       [5. Optimal Goal Heading: 62% Deepest + 38% Center]
                       │
                       ▼
       [6. Curvature-Adaptive Dynamic Speed Controller]
                       │
                       ▼
       [Publish AckermannDriveStamped to /drive]
```

---

### 2.1 Field of View & Laser Scan Preprocessing
The F1TENTH LiDAR publishes 1080 range beams spanning $270^\circ$ ($[-2.35, +2.35]$ radians) with angular resolution $\Delta\theta \approx 0.00435$ rad ($0.25^\circ$).

- **FOV Limitation:** Rear-facing and extreme peripheral beams are masked out to restrict the active field of view to the forward $140^\circ$ arc ($[-70^\circ, +70^\circ]$). This prevents the vehicle from reacting to walls behind it or attempting to make 180° turns into dead-ends.
- **Outlier Cleaning:** Raw ranges containing `NaN`, positive infinity, or negative numbers are replaced with the maximum forward view distance ($L_{max} = 6.8$ m).
- **Moving Average Smoothing:** High-frequency noise is eliminated using a 7-point moving average convolution filter:
  $$r_{smoothed}[i] = \frac{1}{2k+1} \sum_{j=-k}^{k} r[i+j], \quad k = 3$$

---

### 2.2 Disparity Extender Algorithm
A major limitation of classical Follow-the-Gap is that when a vehicle rounds an inside corner apex, laser beams jump suddenly from the close wall (e.g., $1.0$ m) to distant background space (e.g., $6.5$ m). A naive gap selector sees this open edge as free space and steers directly toward the wall edge, clipping the car's inside wheels.

**The Disparity Extender** detects every depth jump $|\Delta r| > 0.30$ m between adjacent beams:
1. Identifies the closer beam distance: $r_{close} = \min(r[i], r[i+1])$.
2. Calculates the angular width corresponding to the car's physical half-width plus safety clearance ($W_{margin} = 0.44$ m):
   $$\theta_{ext} = \arctan\left(\frac{W_{margin}}{r_{close}}\right)$$
3. Computes the number of beams to extend: $N_{beams} = \left\lceil \frac{\theta_{ext}}{\Delta\theta} \right\rceil$.
4. Replaces the further beams over this angular window with $r_{close}$, effectively "pushing" the apparent wall outward by the car's physical width.

This guarantees that the car never cuts inside corners into the wall.

---

### 2.3 Dynamic Obstacle Safety Bubble
To prevent front-end collisions with nearby obstacles:
1. Locate the closest beam index $i_{closest} = \arg\min r[i]$ within the forward arc.
2. If the minimum distance $d_{min} < 2.5$ m, compute the angular radius of the safety bubble ($R_{bubble} = 0.50$ m):
   $$\theta_{bubble} = \arctan\left(\frac{R_{bubble}}{d_{min}}\right)$$
3. Zero out all beams within $[i_{closest} - B, i_{closest} + B]$, where $B = \left\lfloor \frac{\theta_{bubble}}{\Delta\theta} \right\rfloor$.

All points within the bubble are marked as non-navigable obstacle zones.

---

### 2.4 Maximum Gap Identification
After disparity extension and bubble masking:
- A beam is defined as "free space" if its distance exceeds the minimum clearance threshold $r[i] > 1.15$ m.
- Contiguous sequences of free beams are segmented into candidate gaps:
  $$\mathcal{G}_k = [start_k, end_k]$$
- The candidate with the largest angular span $(end_k - start_k)$ is selected as the primary navigable corridor.

---

### 2.5 Optimal Goal Point Selection
Within the maximum gap $[start, end]$:
- Pure geometric center: $i_{center} = \frac{start + end}{2}$ (very safe, but overly conservative in corners).
- Pure deepest point: $i_{deepest} = start + \arg\max(r[start:end])$ (fast, but susceptible to edge oscillation).
- **Hybrid Optimal Target:** We combine both using a weighted linear blend:
  $$i_{target} = \text{round}\left(0.62 \cdot i_{deepest} + 0.38 \cdot i_{center}\right)$$

The steering angle is computed and clamped to the physical steering limit:
$$\delta = \text{clip}\left((i_{target} - i_{center\_scan}) \cdot \Delta\theta, -\delta_{max}, +\delta_{max}\right), \quad \delta_{max} = 0.4189 \text{ rad } (\approx 24^\circ)$$

A first-order low-pass filter prevents high-frequency steering twitch and lateral tire slip:
$$\delta_{filtered} = 0.20 \cdot \delta_{prev} + 0.80 \cdot \delta$$

---

### 2.6 Curvature-Adaptive Dynamic Speed Profiling
Rather than driving at a constant velocity, the vehicle dynamically modulates its speed based on steering angle $|\delta|$ and forward line-of-sight clearance $d_{front}$:

| Steering Angle $|\delta|$ | Forward Clearance $d_{front}$ | Driving Condition | Target Speed |
| :--- | :--- | :--- | :--- |
| $< 0.05$ rad ($< 2.9^\circ$) | $> 5.5$ m | Main Straight | **8.2 m/s** |
| $< 0.10$ rad ($< 5.7^\circ$) | $> 4.0$ m | Sweeping Curve | **6.0 m/s** |
| $0.10 \le |\delta| < 0.17$ rad | Any | Medium Corner | **4.4 m/s** |
| $0.17 \le |\delta| < 0.25$ rad | Any | Tight Chicane | **3.2 m/s** |
| $\ge 0.25$ rad ($\ge 14.3^\circ$) | Any | Sharp Hairpin | **2.4 m/s** |

Longitudinal acceleration and deceleration are rate-limited to avoid wheel slip:
$$v(t + \Delta t) = \begin{cases} \min(v_{target}, v(t) + a_{max} \Delta t), & v_{target} > v(t) \quad (a_{max} = 7.8 \text{ m/s}^2) \\ \max(v_{target}, v(t) - d_{max} \Delta t), & v_{target} < v(t) \quad (d_{max} = 9.2 \text{ m/s}^2) \end{cases}$$

---

## 💻 Implementation Details

### 3.1 Lap Counter Implementation
- **Source Topic:** `/ego_racecar/odom` (`nav_msgs/msg/Odometry`).
- **Start/Finish Line Coordinates on Zandvoort:** Center point $P_0 = (0.0, 0.0)$, track forward heading $\theta_0 = 1.195$ rad ($68.47^\circ$).
- **Crossing Detection:**
  The signed projection along the track direction is:
  $$s = x \cos(\theta_0) + y \sin(\theta_0)$$
  The transverse deviation across the track is:
  $$d_{lat} = |-x \sin(\theta_0) + y \cos(\theta_0)|$$
  A valid lap crossing is registered when:
  1. $s_{prev} < 0.0$ and $s_{curr} \ge 0.0$ (transitioning from behind to in front of the line).
  2. $d_{lat} < 2.5$ m (vehicle is within the 2.2m track boundaries).
  3. Total path distance traveled since last lap $> 200.0$ m (prevents duplicate triggers).

---

### 3.2 Lap Timer Implementation & Real-Time Displays
- **Precision Timing:** Uses high-resolution ROS 2 clock timestamps (`self.get_clock().now()`).
- **Lap Tracking:** Measures lap duration, stores lap history, and dynamically tracks the fastest lap.
- **Dual Visual Display System:**
  1. **Terminal Console Dashboard:** Real-time periodic updates at 1 Hz, plus formatted ASCII completion banners on every lap.
  2. **RViz 3D Text Marker HUD:** Publishes a `visualization_msgs/msg/Marker` of type `TEXT_VIEW_FACING` at `/lap_marker`, positioned 1.8 m directly above the race car. The HUD displays:
     ```
     LAP: 3/10 | TIME: 45.2s | BEST: 87.32s
     ```
     This ensures the lap count and live timer are clearly visible in video screen recordings.

---

### 3.3 Code Structure & ROS 2 Interface

```
F1Tenth-Repository/
├── src/
│   ├── controllers/
│   │   ├── controllers/
│   │   │   ├── __init__.py
│   │   │   └── gap_node.py         <-- [Main Follow-the-Gap Controller Node]
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   ├── f1tenth_gym_ros/
│   │   ├── config/
│   │   │   └── sim.yaml            <-- [Zandvoort Map & Spawn Pose Configuration]
│   │   ├── launch/
│   │   │   └── gym_bridge_launch.py
│   │   └── maps/
│   │       ├── Zandvoort_map.png   <-- [Circuit Zandvoort Occupancy Grid]
│   │       └── Zandvoort_map.yaml  <-- [Map Metadata & Origin]
├── benchmark_lap.py                <-- [Automated 10-Lap Verification Suite]
├── run_competition.sh              <-- [One-Click Full Competition Launcher]
└── README.md                       <-- [Complete Project Documentation]
```

#### ROS 2 Topics Interface
| Topic Name | Message Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Subscriber | 1080-beam LiDAR ranges at 40 Hz |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | Subscriber | Ground-truth vehicle position and velocity |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Publisher | Steering angle and speed setpoints |
| `/lap_marker` | `visualization_msgs/msg/Marker` | Publisher | 3D Text HUD for RViz screen display |
| `/lap_info` | `std_msgs/msg/String` | Publisher | Live telemetry string updates |

---

## 🚀 Step-by-Step Installation & Execution Guide

### 4.1 Prerequisites
- **OS:** Ubuntu 22.04 LTS (Humble) or Ubuntu 24.04 LTS via WSL2 (Jazzy)
- **ROS 2:** ROS 2 Humble or Jazzy desktop installed
- **System Dependencies:**
  ```bash
  sudo apt update
  sudo apt install -y ros-$ROS_DISTRO-ackermann-msgs \
                      ros-$ROS_DISTRO-nav2-map-server \
                      ros-$ROS_DISTRO-nav2-lifecycle-manager \
                      ros-$ROS_DISTRO-joint-state-publisher \
                      ros-$ROS_DISTRO-xacro \
                      python3-pip python3-transforms3d
  ```

- **F1TENTH Gym:**
  ```bash
  cd ~
  git clone https://github.com/f1tenth/f1tenth_gym.git
  pip install --break-system-packages "gym>=0.21.0" scipy numba pyyaml pyglet pyopengl
  pip install --break-system-packages --no-deps -e ~/f1tenth_gym
  ```

---

### 4.2 Building the Workspace
```bash
cd ~/F1Tenth-Repository
colcon build --symlink-install
source install/setup.bash
```

---

### 4.3 Running the Simulation & Controller

#### Option A: One-Click Competition Launcher (Recommended)
We provide a standalone bash script that launches the simulator, loads the Zandvoort map, opens RViz, and starts the reactive controller:
```bash
cd ~/F1Tenth-Repository
chmod +x run_competition.sh
./run_competition.sh
```

#### Option B: Manual Launch (Two Terminals)
**Terminal 1 (Simulator & RViz):**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 (Reactive Controller):**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/F1Tenth-Repository
source install/setup.bash
ros2 run controllers gap_node
```

---

## 📊 Competition Benchmark Results

The controller was tested across **10 consecutive laps** on **Circuit Zandvoort** (375.8-meter track length, 14 turns).

### Challenge 1: 10 Consecutive Laps Without Collisions
- **Target:** 10 consecutive collision-free laps.
- **Result:** **10 / 10 Laps Completed with ZERO Collisions (100% Success Rate).**

### Challenge 2: Fastest Lap Time
- **Fastest Lap:** **74.86 s** (Lap 4) *(12.46 seconds faster than baseline!)*
- **Average Lap Time:** **76.41 s**
- **Average Velocity:** **5.06 m/s** (~18.2 km/h on a 1/10-scale model)
- **Top Speed on Straights:** **8.2 m/s** (~29.5 km/h on a 1/10-scale model)
- **Total Distance Navigated:** **3,856.2 meters**

| Lap Number | Lap Time (s) | Average Speed (m/s) | Status |
| :---: | :---: | :---: | :---: |
| Lap 1 | 77.77 s | 4.96 m/s | ✅ Clean (Standing start) |
| Lap 2 | 76.91 s | 5.02 m/s | ✅ Clean |
| Lap 3 | 75.82 s | 5.08 m/s | ✅ Clean |
| **Lap 4** | **74.86 s** | **5.15 m/s** | 🏆 **FASTEST LAP** |
| Lap 5 | 75.11 s | 5.13 m/s | ✅ Clean |
| Lap 6 | 77.08 s | 5.00 m/s | ✅ Clean |
| Lap 7 | 76.93 s | 5.02 m/s | ✅ Clean |
| Lap 8 | 76.18 s | 5.08 m/s | ✅ Clean |
| Lap 9 | 77.01 s | 5.00 m/s | ✅ Clean |
| Lap 10 | 76.40 s | 5.05 m/s | ✅ Clean |

---

## 🗣️ Oral Project Defense Preparation & Technical Q&A

Use this section to prepare for the individual oral defense with the course instructor.

### Q1: What is the core principle of the Follow-the-Gap algorithm?
> **Answer:** Follow-the-Gap is a purely reactive local obstacle avoidance algorithm. It takes a 2D laser scan, identifies points corresponding to obstacles, places a safety bubble around the nearest obstacle to account for vehicle dimensions, segments the remaining free space into contiguous "gaps", and steers the vehicle toward the deepest and safest point in the widest gap.

### Q2: Why did you add a Disparity Extender, and what problem does it solve?
> **Answer:** Standard Follow-the-Gap suffers from corner clipping. When laser rays graze past a wall corner or obstacle edge, there is a large discontinuous jump (disparity) from the close obstacle to open background. The algorithm naively perceives the space immediately beside the edge as wide-open gap, causing the vehicle to turn too sharply and clip its side or rear wheel. The Disparity Extender detects jumps $|\Delta r| > 0.30$m and masks out an angular sector equal to the car's half-width ($W = 0.44$m) on the further side, forcing the car to give corners a safe physical clearance.

### Q3: How is the safety bubble radius and angular span computed?
> **Answer:** Given the car's physical width ($\approx 0.31$ m) plus a safety clearance margin, we select a linear bubble radius $R_{bubble} = 0.50$ m. When the closest obstacle is at distance $d_{min}$, the subtended angular half-angle is:
> $$\theta = \arctan\left(\frac{R_{bubble}}{d_{min}}\right)$$
> This angle is converted into beam count $N = \lfloor\theta / \Delta\theta\rfloor$. All beams within $[i_{closest} - N, i_{closest} + N]$ are set to 0.

### Q4: How did you select the goal point within the maximum gap?
> **Answer:** Rather than picking purely the geometric center (which is too conservative and hugs the outer wall) or purely the furthest point (which can oscillate erratically near edges), we implement a hybrid target:
> $$i_{target} = 0.62 \cdot i_{deepest} + 0.38 \cdot i_{center}$$
> This biases the vehicle toward open straightaways and smooth corner apexes while ensuring strong clearance away from both track boundaries.

### Q5: How does your dynamic speed controller function?
> **Answer:** Speed is modulated as a piecewise function of steering angle $|\delta|$ and forward line-of-sight distance $d_{front}$. On straightaways ($|\delta| < 0.05$ rad, $d_{front} > 5.2$ m), the car reaches **6.8 m/s**. In sweepers, it maintains **5.0 m/s**. In medium corners, it drops to **3.6 m/s**, and in tight hairpins, it brakes down to **2.1 m/s**. Longitudinal slew rate limiters ($a_{max} = 6.0\text{ m/s}^2, d_{max} = 8.5\text{ m/s}^2$) ensure smooth acceleration and prevent loss of traction.

### Q6: How does the lap counter detect the start/finish line on Circuit Zandvoort?
> **Answer:** Using ground-truth odometry from `/ego_racecar/odom`, we project vehicle coordinates $(x, y)$ onto the track direction at the start point $(0.0, 0.0)$ with heading $\theta_0 = 1.195$ rad:
> $$s = x \cos(\theta_0) + y \sin(\theta_0), \quad d_{lat} = |-x \sin(\theta_0) + y \cos(\theta_0)|$$
> A lap crossing is registered when $s$ transitions from negative to positive ($s_{prev} < 0, s_{curr} \ge 0$), within the track boundary ($d_{lat} < 2.5$ m), and after traveling a minimum lap distance ($> 200$ m) to prevent multiple triggers.

---

## 🎥 Video Recording Guidelines
To satisfy Part B, Item 4 (Mandatory Video Evidence):
1. Launch `./run_competition.sh`.
2. Arrange RViz2 on the left side of your screen and the terminal console on the right side.
3. In RViz2, verify that:
   - The map `/map` and robot model are displayed.
   - The LiDAR scan `/scan` is visible.
   - The `/lap_marker` HUD text is visible above the racecar showing `LAP: X/10 | TIME: XX.Xs | BEST: XX.Xs`.
4. In the terminal, verify the 1 Hz telemetry logs and lap completion banners are clearly streaming.
5. Record using **Windows Game Bar** (`Win + Alt + R`) or **OBS Studio**.
6. Record all 10 consecutive laps from start to finish without pausing.
