# Autonomous Vehicles Course: F1TENTH Follow-the-Gap Controller
**Student:** Alamgir Abu Hanif  
**Group:** Group 4  
**Assigned Track:** Circuit Zandvoort (`Zandvoort_map.png`)  
**Repository:** [https://github.com/Ah-Alamgir/assaignment](https://github.com/Ah-Alamgir/assaignment)

---

## 📋 Table of Contents
1. [Project Overview & Objectives](#-project-overview--objectives)
2. [Reactive Controller Architecture (Follow-the-Gap)](#-reactive-controller-architecture-follow-the-gap)
   - [2.1 Field of View & Dual-Horizon Laser Scan Preprocessing](#21-field-of-view--dual-horizon-laser-scan-preprocessing)
   - [2.2 Non-Cascading Vectorized Disparity Extender](#22-non-cascading-vectorized-disparity-extender)
   - [2.3 Dynamic Obstacle Safety Bubble](#23-dynamic-obstacle-safety-bubble)
   - [2.4 Vectorized Maximum Gap Identification](#24-vectorized-maximum-gap-identification)
   - [2.5 Plateau-Stabilized Hybrid Goal Point Selection](#25-plateau-stabilized-hybrid-goal-point-selection)
   - [2.6 7-Stage Curvature-Adaptive & Dual-Cone Velocity Profiling](#26-7-stage-curvature-adaptive--dual-cone-velocity-profiling)
3. [Implementation Details](#-implementation-details)
   - [3.1 Lap Counter Implementation](#31-lap-counter-implementation)
   - [3.2 Lap Timer Implementation & Real-Time Displays](#32-lap-timer-implementation--real-time-displays)
   - [3.3 Code Structure & ROS 2 Interface](#33-code-structure--ros-2-interface)
4. [Step-by-Step Installation & Execution Guide](#-step-by-step-installation--execution-guide)
   - [4.1 Prerequisites](#41-prerequisites)
   - [4.2 Building the Workspace](#42-building-the-workspace)
   - [4.3 Running the Simulation & Controller (`runsim`)](#43-running-the-simulation--controller-runsim)
   - [4.4 Running the Standalone 10-Lap Benchmark](#44-running-the-standalone-10-lap-benchmark)
5. [Competition Benchmark Results](#-competition-benchmark-results)
   - [Challenge 1: 10 Consecutive Laps Without Collisions](#challenge-1-10-consecutive-laps-without-collisions)
   - [Challenge 2: Fastest Lap Time](#challenge-2-fastest-lap-time)
6. [Oral Project Defense Preparation & Technical Q&A](#-oral-project-defense-preparation--technical-qa)
7. [Video Recording Guidelines](#-video-recording-guidelines)

---

## 🎯 Project Overview & Objectives

This project implements a high-performance autonomous reactive navigation system for a 1/10-scale autonomous race car using the official **F1TENTH Simulator** in **ROS 2**. The vehicle is evaluated in a high-speed racing competition on **Circuit Zandvoort**, a technical racetrack characterized by 14 complex corners, high-speed sweepers, tight chicanes, and narrow hairpins.

The assignment evaluates two core competition criteria:
1. **Challenge 1: 10 Consecutive Laps Without Collisions** — The reactive controller must navigate the entire ~377.0-meter track for 10 complete, uninterrupted laps without contacting track boundaries or obstacles.
2. **Challenge 2: Fastest Lap Time** — Achieving the fastest single lap time among the 10 completed laps through disparity extension, plateau-stabilized apex targeting, and dual-cone dynamic velocity scheduling, while displaying real-time lap counts and lap times in both RViz2 and the terminal console.

---

## 🧠 Reactive Controller Architecture (Follow-the-Gap)

The Follow-the-Gap (FTG) method is a purely reactive obstacle avoidance algorithm operating directly on 2D laser rangefinder (LiDAR) scans. Rather than relying on pre-recorded global waypoints or SLAM localization at runtime, FTG continuously evaluates surrounding track geometry to detect navigable free space ("gaps") and steer the vehicle along the optimal collision-free corridor.

```
        [Raw LiDAR Scan: 1080 beams, 270° FOV]
                         │
                         ▼
        [1. 135° Forward FOV & 5-Point Moving Avg]
        [   Dual Horizon: 30m Straight / 12m Gap ]
                         │
                         ▼
        [2. Non-Cascading Vectorized Disparity Extender]
                         │
                         ▼
        [3. Safety Bubble Inflation around Closest Point]
                         │
                         ▼
        [4. Vectorized Contiguous Free-Space Gap Finder]
                         │
                         ▼
        [5. Plateau-Stabilized Hybrid Goal Selection]
        [   76% Deepest Plateau Centroid + 24% Center]
                         │
                         ▼
        [6. 7-Stage Dual-Cone Speed & Braking Horizon]
                         │
                         ▼
        [Publish AckermannDriveStamped to /drive]
```

---

### 2.1 Field of View & Dual-Horizon Laser Scan Preprocessing
The F1TENTH Hokuyo LiDAR publishes 1080 range beams spanning $270^\circ$ ($[-2.35, +2.35]$ radians) with an angular resolution of $\Delta\theta \approx 0.00435$ rad ($0.25^\circ$).

- **FOV Limitation:** Rear-facing and extreme peripheral beams are masked out to restrict the active field of view to the forward $135^\circ$ arc ($[-67.5^\circ, +67.5^\circ]$). This prevents the vehicle from reacting to walls behind it or attempting U-turns in tight hairpins.
- **Outlier Sanitization:** Raw readings containing `NaN` or negative values are replaced with `0.0`, while positive infinity readings are clamped to `30.0` m.
- **Moving Average Smoothing:** High-frequency sensor noise is suppressed using a 5-point moving average convolution filter ($k = 2$):
  $$r_{\text{smoothed}}[i] = \frac{1}{5} \sum_{j=-2}^{2} r[i+j]$$
- **Dual-Horizon Clipping:**
  - **Long-Range Horizon ($L_{\text{long}} = 30.0\text{ m}$):** Preserves unclipped straightaway depth for early high-speed acceleration and braking lookahead.
  - **Local Gap Horizon ($L_{\text{max}} = 12.0\text{ m}$):** Clips ranges used for local gap segmentation so distant track sections do not skew immediate corner entry geometry.

---

### 2.2 Non-Cascading Vectorized Disparity Extender
A classic failure mode of standard Follow-the-Gap occurs when rounding an inside corner apex: laser beams jump discontinuously from the near inside wall (e.g., $1.0$ m) to distant open background (e.g., $10.0$ m). A naive gap finder perceives the space immediately adjacent to the corner edge as open and steers straight into the apex, clipping the car's inside wheels.

**Our Non-Cascading Disparity Extender** inspects unmodified preprocessed ranges to detect every true depth jump $|\Delta r| = |r[i] - r[i+1]| > 0.28$ m within the active FOV:
1. Identifies the closer obstacle distance: $r_{\text{close}} = \min(r[i], r[i+1])$.
2. Calculates the angular safety margin corresponding to the car's effective half-width plus clearance ($W_{\text{margin}} = 0.45$ m):
   $$\theta_{\text{ext}} = \arctan\left(\frac{W_{\text{margin}}}{r_{\text{close}}}\right)$$
3. Computes the number of laser beams to mask:
   $$N_{\text{beams}} = \left\lfloor \frac{\theta_{\text{ext}}}{\Delta\theta} \right\rfloor$$
4. Clamps the further beams across those $N_{\text{beams}}$ indices down to $\min(r_{\text{ext}}, r_{\text{close}})$ without cascading into subsequent disparity checks.

This guarantees that the vehicle always leaves sufficient lateral clearance around inside corners and chicanes.

---

### 2.3 Dynamic Obstacle Safety Bubble
To protect against close-range obstacles and wall proximity:
1. Locate the closest beam index $i_{\text{closest}} = \arg\min r[i]$ within the forward FOV $[i_{\min}, i_{\max}]$.
2. If the minimum distance satisfies $0.05\text{ m} < d_{\min} < 2.5\text{ m}$, compute the angular subtense of the safety bubble ($R_{\text{bubble}} = 0.48$ m):
   $$\theta_{\text{bubble}} = \arctan\left(\frac{R_{\text{bubble}}}{d_{\min}}\right)$$
3. Zero out all beams within $[i_{\text{closest}} - B, i_{\text{closest}} + B]$, where $B = \left\lfloor \frac{\theta_{\text{bubble}}}{\Delta\theta} \right\rfloor$.

All zeroed beams are treated as impassable obstacle regions during gap extraction.

---

### 2.4 Vectorized Maximum Gap Identification
After disparity extension and safety bubble masking:
- A beam is classified as navigable free space if its range exceeds the clearance threshold $r[i] > 1.35$ m.
- Using vectorized NumPy boolean transition detection (`np.flatnonzero`), all contiguous sequences of free beams are segmented into candidate gaps:
  $$\mathcal{G}_k = [\text{start}_k, \text{end}_k]$$
- The contiguous segment with the maximum angular width $(\text{end}_k - \text{start}_k)$ is selected as the primary navigable corridor.

---

### 2.5 Plateau-Stabilized Hybrid Goal Point Selection
Selecting a single maximum-range beam inside a clipped gap often causes high-frequency steering oscillation because multiple beams hit the $12.0$ m horizon cap simultaneously. Conversely, selecting only the geometric gap center is overly conservative and forces wide corner lines.

We implement a **Plateau-Stabilized Hybrid Target**:
1. Identify all beams inside the maximum gap $[\text{start}, \text{end}]$ whose range is within $97\%$ of the gap's peak depth:
   $$\mathcal{P} = \left\{ i \in [\text{start}, \text{end}] \;\middle|\; r[i] \ge 0.97 \cdot \max_{j \in [\text{start}, \text{end}]} r[j] \right\}$$
2. Compute the deepest plateau centroid $i_{\text{deepest}} = \text{mean}(\mathcal{P})$ and the geometric gap center $i_{\text{center}} = \lfloor(\text{start} + \text{end}) / 2\rfloor$.
3. Blend both indices using weight $w = 0.76$:
   $$i_{\text{target}} = \text{int}\left(0.76 \cdot i_{\text{deepest}} + 0.24 \cdot i_{\text{center}}\right)$$
4. Convert the target index into a raw steering angle with proportional gain $K_p = 1.06$ and clamp to the physical steering limit $\delta_{\max} = 0.4189\text{ rad}$ ($\approx 24^\circ$):
   $$\delta_{\text{raw}} = \text{clip}\left((i_{\text{target}} - i_{\text{center\_scan}}) \cdot \Delta\theta \cdot K_p,\; -\delta_{\max},\; +\delta_{\max}\right)$$
5. Apply an exponential low-pass filter ($\alpha = 0.88$) for smooth actuator response:
   $$\delta_{\text{filtered}} = (1 - \alpha)\,\delta_{\text{prev}} + \alpha\,\delta_{\text{raw}} = 0.12\,\delta_{\text{prev}} + 0.88\,\delta_{\text{raw}}$$

---

### 2.6 7-Stage Curvature-Adaptive & Dual-Cone Velocity Profiling
To maximize speed on Zandvoort's long front straight while braking safely before tight hairpins, the controller evaluates a **dual-cone lookahead distance** using the unclipped $30\text{ m}$ ranges:
- **Forward Cone ($d_{\text{front}}$):** Minimum distance across $[i_{\text{center}} - 10, i_{\text{center}} + 10]$ ($\approx \pm 2.5^\circ$ straight ahead).
- **Target Cone ($d_{\text{target}}$):** Minimum distance across $[i_{\text{target}} - 6, i_{\text{target}} + 6]$ along the commanded steering trajectory.
- **Effective Lookahead Distance ($d_{\text{eff}}$):**
  $$d_{\text{eff}} = 0.75 \cdot d_{\text{front}} + 0.25 \cdot d_{\text{target}}$$

| Stage | Steering Angle $\|\delta\|$ | Clearance Condition | Driving Regime | Target Speed |
| :---: | :--- | :--- | :--- | :--- |
| **1** | $< 0.030$ rad ($< 1.7^\circ$) | $d_{\text{front}} > 11.0$ m | Long Main Straight (`v_ultra`) | **10.5 m/s** (37.8 km/h) |
| **2** | $< 0.045$ rad ($< 2.6^\circ$) | $d_{\text{front}} > 8.0$ m | Standard Straight (`v_max`) | **9.5 m/s** (34.2 km/h) |
| **3** | $< 0.075$ rad ($< 4.3^\circ$) | $d_{\text{eff}} > 5.5$ m | Fast Kink / Corner Exit (`v_high`) | **8.0 m/s** (28.8 km/h) |
| **4** | $< 0.115$ rad ($< 6.6^\circ$) | $d_{\text{eff}} > 3.8$ m | Sweeping Curve (`v_sweeper`) | **6.6 m/s** (23.8 km/h) |
| **5** | $< 0.175$ rad ($< 10.0^\circ$) | $d_{\text{eff}} > 2.6$ m | Medium Corner (`v_medium`) | **5.2 m/s** (18.7 km/h) |
| **6** | $< 0.255$ rad ($< 14.6^\circ$) | Any | Tight Chicane (`v_tight`) | **4.0 m/s** (14.4 km/h) |
| **7** | $\ge 0.255$ rad ($\ge 14.6^\circ$) | Any | Sharp Hairpin Apex (`v_min`) | **3.1 m/s** (11.2 km/h) |

Finally, a **kinematic forward braking horizon cap** ensures the vehicle always decelerates in time when approaching an oncoming wall at the end of a high-speed straight:
$$v_{\text{cmd}} = \min\left(v_{\text{target}},\; \max\left(v_{\min},\; \sqrt{2 \cdot a_{\text{brake}} \cdot \max(0.15,\; d_{\text{eff}} - d_{\text{margin}})}\right)\right)$$
where $a_{\text{brake}} = 4.8\text{ m/s}^2$ and $d_{\text{margin}} = 1.30\text{ m}$.

---

## 💻 Implementation Details

### 3.1 Lap Counter Implementation
- **Odometry Source:** Subscribes to `/ego_racecar/odom` (`nav_msgs/msg/Odometry`).
- **Start/Finish Line Pose on Circuit Zandvoort:** Origin $P_0 = (0.0, 0.0)$ with forward track heading $\theta_0 = 1.195\text{ rad}$ ($68.47^\circ$).
- **Orthogonal Line Crossing Detection:**
  The longitudinal projection along the track tangent and lateral offset across the track width are computed at every odometry update:
  $$s = x \cos(\theta_0) + y \sin(\theta_0), \qquad d_{\text{lat}} = |-x \sin(\theta_0) + y \cos(\theta_0)|$$
  A completed lap is registered when all three conditions hold simultaneously:
  1. $s_{\text{prev}} < 0.0$ and $s_{\text{curr}} \ge 0.0$ (forward crossing across the start/finish line).
  2. $d_{\text{lat}} < 2.5\text{ m}$ (vehicle is within the track corridor around the start line).
  3. Cumulative trajectory distance traveled since the last lap trigger exceeds $200.0\text{ m}$ (eliminates false re-triggers near the origin).

---

### 3.2 Lap Timer Implementation & Real-Time Displays
- **High-Resolution Timing:** Uses the ROS 2 node clock (`self.get_clock().now()`) triggered automatically upon the vehicle's first movement.
- **Dual Real-Time Display Architecture:**
  1. **RViz Top-Right Corner Scoreboard & Car HUD (`/lap_marker`):**
     - Publishes `visualization_msgs/msg/Marker` (`TEXT_VIEW_FACING`) in the `map` frame.
     - **Top-Right Scoreboard (`id = 1, 10..13`):** Rendered in bold black text at map coordinates $(56.5, -34.0, 2.5)$ — aligned with the top-right corner of the pre-configured overhead RViz camera (`gym_bridge.rviz`) — listing every completed lap and the live active lap:
       ```text
       lap 1- 53.5sec |no colition
       lap 2- 53.0sec |no colition
       ...
       lap 7- 52.8sec |no colition
       ```
     - **Dynamic Car Follower Tag (`id = 2`):** Rendered $2.0\text{ m}$ directly above the moving racecar in bright yellow text showing the current lap and live elapsed timer.
  2. **Live 10 Hz Console Telemetry & Lap Summary Banners:**
     - Streams a real-time 10 Hz status bar in the terminal showing current lap, elapsed lap timer, instantaneous speed ($\text{m/s}$ and $\text{km/h}$), best lap record, and distance covered.
     - Prints detailed summary banners at the end of each lap and a complete 10-lap championship report once Lap 10 finishes.

---

### 3.3 Code Structure & ROS 2 Interface

```text
assaignment/
├── src/
│   ├── controllers/
│   │   ├── controllers/
│   │   │   ├── __init__.py
│   │   │   └── gap_node.py              # Main Follow-the-Gap Controller + Lap Counter/Timer/HUD
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   ├── f1tenth_gym_ros/
│   │   ├── config/
│   │   │   └── sim.yaml                 # Zandvoort Map Path & Spawn Pose (0.0, 0.0, 1.195)
│   │   ├── f1tenth_gym_ros/
│   │   │   └── gym_bridge.py            # F1TENTH Gym <-> ROS 2 Bridge Node
│   │   ├── launch/
│   │   │   ├── gym_bridge_launch.py     # Launches Bridge, Map Server, RSP, and RViz2
│   │   │   └── gym_bridge.rviz          # Pre-configured Full-Track Zandvoort RViz Layout + HUD
│   │   └── maps/
│   │       ├── Zandvoort_map.png        # Circuit Zandvoort Occupancy Grid Image
│   │       └── Zandvoort_map.yaml       # Map Resolution & Origin Metadata
│   ├── path_planning/                   # Auxiliary path planning package
│   └── waypoints/
│       ├── Zandvoort_centerline.csv     # Circuit Zandvoort reference centerline
│       └── levine_path.csv
├── Laboratories/                        # Course laboratory guides (Labs 1-7)
├── benchmark_lap.py                     # Standalone headless 10-lap physics benchmark script
├── runsim                               # One-command Linux/WSL2 simulation + RViz + controller runner
├── runsim.bat                           # One-command Windows CMD launcher
├── runsim.ps1                           # One-command Windows PowerShell launcher
└── README.md                            # Project documentation & defense guide
```

#### ROS 2 Topics Interface
| Topic Name | Message Type | Role in `gap_node` | Description |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Subscriber | 1080-beam 2D LiDAR range array |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | Subscriber | Vehicle position, orientation, and twist |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Publisher | Commanded steering angle ($\text{rad}$) and speed ($\text{m/s}$) |
| `/lap_marker` | `visualization_msgs/msg/Marker` | Publisher | Top-right RViz scoreboard and car-following 3D text HUD |
| `/lap_info` | `std_msgs/msg/String` | Publisher | Formatted 10 Hz live telemetry string |

---

## 🚀 Step-by-Step Installation & Execution Guide

### 4.1 Prerequisites
- **Operating System:** Ubuntu 22.04 LTS (ROS 2 Humble) or Ubuntu 24.04 LTS via WSL2 (ROS 2 Jazzy)
- **System & ROS 2 Dependencies:**
  ```bash
  sudo apt update
  sudo apt install -y ros-$ROS_DISTRO-ackermann-msgs \
                      ros-$ROS_DISTRO-nav2-map-server \
                      ros-$ROS_DISTRO-nav2-lifecycle-manager \
                      ros-$ROS_DISTRO-joint-state-publisher \
                      ros-$ROS_DISTRO-xacro \
                      python3-pip python3-transforms3d x11-utils
  ```
- **F1TENTH Gym Environment:**
  ```bash
  cd ~
  git clone https://github.com/f1tenth/f1tenth_gym.git
  pip install --break-system-packages "gym>=0.21.0" scipy numba pyyaml pyglet pyopengl
  pip install --break-system-packages --no-deps -e ~/f1tenth_gym
  ```

---

### 4.2 Building the Workspace
```bash
cd /mnt/d/assaignment
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

### 4.3 Running the Simulation & Controller (`runsim`)

#### Option A: One-Command Launcher (Recommended)
The [`runsim`](file:///d:/assaignment/runsim) script automatically cleans up any stale ROS 2 processes, builds the workspace if needed, launches the F1TENTH simulator and RViz2 framed on Circuit Zandvoort, and runs `gap_node` with live console telemetry:

- **From Linux / WSL2 Terminal:**
  ```bash
  cd /mnt/d/assaignment
  ./runsim
  ```
- **From Windows PowerShell:**
  ```powershell
  .\runsim.ps1
  ```
- **From Windows Command Prompt (CMD):**
  ```cmd
  runsim.bat
  ```

#### Option B: Manual Launch (Two Terminals)
**Terminal 1 — Launch Simulator & RViz2:**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /mnt/d/assaignment
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

**Terminal 2 — Launch Follow-the-Gap Controller:**
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd /mnt/d/assaignment
source install/setup.bash
ros2 run controllers gap_node
```

---

### 4.4 Running the Standalone 10-Lap Benchmark
To verify all 10 consecutive laps rapidly in headless physics mode ($100\text{ Hz}$ RK4 integration):
```bash
cd /mnt/d/assaignment
python3 benchmark_lap.py
```

---

## 📊 Competition Benchmark Results

The controller was benchmarked across **10 consecutive laps** on **Circuit Zandvoort** (~377.0-meter lap distance, 14 turns) starting from $(0.0, 0.0, 1.195\text{ rad})$.

### Challenge 1: 10 Consecutive Laps Without Collisions
- **Requirement:** Complete 10 consecutive laps without any wall or obstacle collisions.
- **Result:** **10 / 10 Laps Completed — ZERO Collisions (100% Reliability).**

### Challenge 2: Fastest Lap Time
- **Fastest Single Lap:** **52.81 s** (Lap 7) *(34.51 seconds faster than the initial 87.32 s baseline!)*
- **Average Lap Time (10 Laps):** **52.99 s** (Standard Deviation: $\pm 0.18\text{ s}$)
- **Total 10-Lap Race Time:** **529.86 s**
- **Average Lap Velocity:** **7.12 m/s** (~25.6 km/h)
- **Peak Straightaway Velocity:** **10.50 m/s** (~37.8 km/h)
- **Total Distance Driven:** **3,770.1 meters**

| Lap Number | Lap Time (s) | Lap Distance (m) | Average Speed | RViz Top-Right HUD Scoreboard |
| :---: | :---: | :---: | :---: | :--- |
| Lap 1 | 53.48 s | 377.1 m | 7.05 m/s (25.4 km/h) | `lap 1- 53.5sec \|no colition` |
| Lap 2 | 52.96 s | 377.0 m | 7.12 m/s (25.6 km/h) | `lap 2- 53.0sec \|no colition` |
| Lap 3 | 53.06 s | 377.0 m | 7.11 m/s (25.6 km/h) | `lap 3- 53.1sec \|no colition` |
| Lap 4 | 52.86 s | 377.0 m | 7.13 m/s (25.7 km/h) | `lap 4- 52.9sec \|no colition` |
| Lap 5 | 52.90 s | 377.0 m | 7.13 m/s (25.7 km/h) | `lap 5- 52.9sec \|no colition` |
| Lap 6 | 52.89 s | 377.0 m | 7.13 m/s (25.7 km/h) | `lap 6- 52.9sec \|no colition` |
| **Lap 7** | **52.81 s** | **377.0 m** | **7.14 m/s (25.7 km/h)** | 🏆 `lap 7- 52.8sec \|no colition` |
| Lap 8 | 53.10 s | 377.0 m | 7.10 m/s (25.6 km/h) | `lap 8- 53.1sec \|no colition` |
| Lap 9 | 52.90 s | 377.0 m | 7.13 m/s (25.7 km/h) | `lap 9- 52.9sec \|no colition` |
| Lap 10 | 52.90 s | 377.0 m | 7.13 m/s (25.7 km/h) | `lap 10- 52.9sec \|no colition` |

---

## 🗣️ Oral Project Defense Preparation & Technical Q&A

Use this section to prepare for the individual oral defense with the course instructor.

### Q1: What is the core principle of the Follow-the-Gap algorithm?
> **Answer:** Follow-the-Gap is a reactive local navigation algorithm that operates directly on 2D LiDAR scans without requiring a global path planner. At each control step, it restricts the scan to the forward field of view ($135^\circ$), inflates obstacle edges via a Disparity Extender and a close-range Safety Bubble to account for vehicle width, segments the remaining free space ($r > 1.35\text{ m}$) into contiguous gaps, and steers toward an optimal target inside the widest gap.

### Q2: Why did you implement a Non-Cascading Disparity Extender, and what problem does it solve?
> **Answer:** Standard Follow-the-Gap suffers from inside-corner clipping: when laser rays graze past an inside wall apex, there is a sharp depth jump from the near wall to open background. Without disparity extension, the car steers directly toward that edge and clips its rear wheel. Our Non-Cascading Disparity Extender detects jumps $|\Delta r| > 0.28\text{ m}$ on the unmodified preprocessed scan and masks out an angular sector $\theta_{\text{ext}} = \arctan(0.45 / r_{\text{close}})$ on the further side, forcing the trajectory to clear every corner apex safely.

### Q3: How is the safety bubble radius and angular span computed?
> **Answer:** Given the car's width plus safety clearance, we set a bubble radius $R_{\text{bubble}} = 0.48\text{ m}$. When the closest obstacle within the forward FOV is at distance $d_{\min} < 2.5\text{ m}$, the angular half-width is $\theta_{\text{bubble}} = \arctan(0.48 / d_{\min})$, corresponding to $B = \lfloor \theta_{\text{bubble}} / \Delta\theta \rfloor$ laser beams on each side of $i_{\text{closest}}$, which are set to zero.

### Q4: How do you select the steering target point inside the maximum gap without oscillation?
> **Answer:** Because local gap ranges are clipped at $12.0\text{ m}$, multiple beams in a straightaway or open curve share the maximum depth, causing single-beam `argmax` selectors to jitter. Instead, we extract the **deepest plateau** (all beams within $\ge 97\%$ of the maximum depth in the gap), compute its mean index $i_{\text{deepest}}$, and blend it with the gap's geometric center $i_{\text{center}}$:
> $$i_{\text{target}} = 0.76 \cdot i_{\text{deepest}} + 0.24 \cdot i_{\text{center}}$$
> We then apply a proportional gain $K_p = 1.06$ and a low-pass filter ($\delta = 0.12\,\delta_{\text{prev}} + 0.88\,\delta_{\text{raw}}$) for smooth cornering.

### Q5: How does your dynamic speed controller achieve ~52.8 s laps without skidding or crashing?
> **Answer:** We use a **dual-horizon, dual-cone velocity controller**. While gap finding uses a $12.0\text{ m}$ horizon, speed control inspects unclipped $30.0\text{ m}$ ranges across both a forward cone ($d_{\text{front}}$) and a steering-aligned cone ($d_{\text{target}}$), forming $d_{\text{eff}} = 0.75\,d_{\text{front}} + 0.25\,d_{\text{target}}$. Across 7 steering curvature stages, speed scales from **10.5 m/s** on Zandvoort's main straight down to **3.1 m/s** in tight hairpins. Simultaneously, a kinematic braking horizon cap $v \le \sqrt{2 \cdot 4.8 \cdot (d_{\text{eff}} - 1.30)}$ automatically initiates braking before corner entry.

### Q6: How does the lap counter accurately detect the start/finish line on Circuit Zandvoort?
> **Answer:** Using `/ego_racecar/odom`, we project the car's position $(x, y)$ onto the start/finish line at $(0.0, 0.0)$ oriented along $\theta_0 = 1.195\text{ rad}$:
> $$s = x \cos(\theta_0) + y \sin(\theta_0), \qquad d_{\text{lat}} = |-x \sin(\theta_0) + y \cos(\theta_0)|$$
> A lap is counted when $s$ crosses from negative to non-negative ($s_{\text{prev}} < 0 \land s_{\text{curr}} \ge 0$), within track bounds ($d_{\text{lat}} < 2.5\text{ m}$), after accumulating at least $200.0\text{ m}$ of driving distance since the previous lap.

---

## 🎥 Video Recording Guidelines
To record the competition demonstration video:
1. Run `./runsim` (or `.\runsim.ps1` from Windows PowerShell).
2. Place **RViz2** on the left side of the screen and the **terminal console** on the right side.
3. Verify that in RViz2:
   - The full **Circuit Zandvoort** map (`/map`), robot model, and LiDAR scan (`/scan`) are visible.
   - The **Top-Right Scoreboard** (`LapCounterHUD` on `/lap_marker`) is clearly visible in bold black text showing `lap 1- XX.Xsec |no colition`, `lap 2- XX.Xsec |no colition`, etc., along with the yellow live tag above the racecar.
4. Verify that in the terminal, the 10 Hz live telemetry bar and lap completion banners are updating in real time.
5. Record all 10 consecutive collision-free laps from start to finish using **Windows Game Bar** (`Win + Alt + R`) or **OBS Studio**.
