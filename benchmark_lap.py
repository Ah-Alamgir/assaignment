import gym
import f110_gym
import numpy as np
import time

class OptimizedFollowTheGapController:
    def __init__(self):
        # Physical parameters
        self.bubble_radius = 0.49       # Safety bubble radius (m)
        self.max_lidar_dist = 8.8       # Extended forward horizon (m)
        self.fov_deg = 145.0            # Field of view (+/- 72.5 deg)
        self.min_gap_dist = 1.12        # Free space threshold
        self.disparity_threshold = 0.28 # Disparity jump threshold
        self.car_width_margin = 0.43    # Effective car margin for disparity
        
        # High-performance balanced dynamics
        self.current_speed = 0.0
        self.prev_steer = 0.0
        self.max_speed = 8.5            # Top speed on straights (m/s)
        self.sweeper_speed = 5.8        # Sweepers (m/s)
        self.medium_turn_speed = 4.0    # Medium corners (m/s)
        self.tight_turn_speed = 2.9     # Chicanes / tight turns (m/s)
        self.min_speed = 2.2            # Hairpin apex speed (m/s)
        
        self.max_accel = 7.5            # Acceleration rate (m/s^2)
        self.max_decel = 9.2            # Deceleration rate (m/s^2)
        self.dt = 0.01

    def process_scan(self, raw_ranges, angle_min, angle_inc):
        ranges = np.array(raw_ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_lidar_dist, neginf=0.0)
        
        half_fov = np.radians(self.fov_deg / 2.0)
        center_idx = int((-angle_min) / angle_inc)
        span_idx = int(half_fov / angle_inc)
        min_i = max(0, center_idx - span_idx)
        max_i = min(len(ranges), center_idx + span_idx)
        
        # 5-point moving average filter
        kernel_size = 5
        smoothed = np.convolve(ranges, np.ones(kernel_size) / kernel_size, mode='same')
        proc_ranges = np.clip(smoothed, 0.0, self.max_lidar_dist)
        
        # Disparity extender
        ext_ranges = np.copy(proc_ranges)
        for i in range(min_i, max_i - 1):
            diff = ext_ranges[i] - ext_ranges[i + 1]
            if abs(diff) > self.disparity_threshold:
                if diff > 0:
                    r_close = ext_ranges[i + 1]
                    if r_close > 0.05:
                        ang_ext = np.arctan(self.car_width_margin / r_close)
                        n_beams = int(ang_ext / angle_inc)
                        start_k = max(min_i, i - n_beams + 1)
                        ext_ranges[start_k:i + 1] = np.minimum(ext_ranges[start_k:i + 1], r_close)
                else:
                    r_close = ext_ranges[i]
                    if r_close > 0.05:
                        ang_ext = np.arctan(self.car_width_margin / r_close)
                        n_beams = int(ang_ext / angle_inc)
                        end_k = min(max_i, i + 1 + n_beams)
                        ext_ranges[i + 1:end_k] = np.minimum(ext_ranges[i + 1:end_k], r_close)

        # Safety bubble
        closest_idx = min_i + np.argmin(ext_ranges[min_i:max_i])
        min_dist = ext_ranges[closest_idx]
        if min_dist < 2.5 and min_dist > 0.05:
            bubble_rad = self.bubble_radius
            bubble_angle = np.arctan(bubble_rad / min_dist)
            b_idx = int(bubble_angle / angle_inc)
            b_start = max(min_i, closest_idx - b_idx)
            b_end = min(max_i, closest_idx + b_idx + 1)
            ext_ranges[b_start:b_end] = 0.0

        # Find gaps
        free_mask = ext_ranges[min_i:max_i] > self.min_gap_dist
        gaps = []
        in_gap = False
        start = 0
        for idx, val in enumerate(free_mask):
            if val and not in_gap:
                in_gap = True
                start = idx
            elif not val and in_gap:
                in_gap = False
                gaps.append((start + min_i, idx + min_i))
        if in_gap:
            gaps.append((start + min_i, max_i))

        if not gaps:
            target_idx = center_idx
        else:
            best_gap = max(gaps, key=lambda g: g[1] - g[0])
            g_start, g_end = best_gap
            gap_slice = ext_ranges[g_start:g_end]
            deepest_sub_idx = g_start + np.argmax(gap_slice)
            center_sub_idx = (g_start + g_end) // 2
            target_idx = int(0.66 * deepest_sub_idx + 0.34 * center_sub_idx)

        raw_steer = (target_idx - center_idx) * angle_inc
        raw_steer = float(np.clip(raw_steer, -0.4189, 0.4189))

        steer = 0.18 * self.prev_steer + 0.82 * raw_steer
        self.prev_steer = steer

        # Dynamic speed profiling
        abs_steer = abs(steer)
        front_dist = ext_ranges[center_idx]
        if abs_steer < 0.05 and front_dist > 5.8:
            target_speed = self.max_speed
        elif abs_steer < 0.10 and front_dist > 4.0:
            target_speed = self.sweeper_speed
        elif abs_steer < 0.17:
            target_speed = self.medium_turn_speed
        elif abs_steer < 0.25:
            target_speed = self.tight_turn_speed
        else:
            target_speed = self.min_speed

        if target_speed > self.current_speed:
            self.current_speed = min(target_speed, self.current_speed + self.max_accel * self.dt)
        else:
            self.current_speed = max(target_speed, self.current_speed - self.max_decel * self.dt)

        return steer, self.current_speed


def run_benchmark(target_laps=10):
    print(f"--- Running {target_laps} Laps High-Performance Optimized Benchmark on Zandvoort ---")
    map_path = '/home/abuhanif/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Zandvoort_map'
    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=1, disable_env_checker=True).unwrapped
    
    obs, _, done, _ = env.reset(np.array([[0.0, 0.0, 1.195]]))

    controller = OptimizedFollowTheGapController()
    scan_fov = 4.7
    scan_beams = 1080
    angle_min = -scan_fov / 2.0
    angle_inc = scan_fov / scan_beams

    theta0 = 1.195
    cos_t0 = np.cos(theta0)
    sin_t0 = np.sin(theta0)

    step = 0
    prev_s = 0.0
    total_lap_dist = 0.0
    prev_x, prev_y = 0.0, 0.0
    lap_start_time = 0.0
    completed_laps = 0
    lap_times = []

    start_wall_time = time.time()

    while not env.collisions[0] and completed_laps < target_laps:
        scans = obs['scans'][0]
        steer, speed = controller.process_scan(scans, angle_min, angle_inc)
        obs, _, done, _ = env.step(np.array([[steer, speed]]))
        step += 1

        x = obs['poses_x'][0]
        y = obs['poses_y'][0]
        dx = x - prev_x
        dy = y - prev_y
        total_lap_dist += np.hypot(dx, dy)
        prev_x, prev_y = x, y

        s_curr = x * cos_t0 + y * sin_t0
        d_lat = abs(-x * sin_t0 + y * cos_t0)
        curr_sim_time = step * 0.01

        if prev_s < 0.0 and s_curr >= 0.0 and d_lat < 2.5 and total_lap_dist > 200.0:
            completed_laps += 1
            lap_time = curr_sim_time - lap_start_time
            lap_times.append(lap_time)
            avg_speed = total_lap_dist / lap_time
            print(f"🏁 Lap {completed_laps:2d}/{target_laps} Completed | Time: {lap_time:6.2f} s | Avg Speed: {avg_speed:4.2f} m/s | Dist: {total_lap_dist:5.1f} m")
            lap_start_time = curr_sim_time
            total_lap_dist = 0.0

        prev_s = s_curr

    elapsed_wall = time.time() - start_wall_time
    print("\n" + "="*50)
    if env.collisions[0]:
        print(f"❌ COLLISION OCCURRED at step {step} (sim time {step*0.01:.2f} s)!")
        print(f"   Car position: ({x:.2f}, {y:.2f})")
    elif completed_laps >= target_laps:
        print(f"🏆 SUCCESS: Completed {completed_laps} consecutive laps WITHOUT any collisions!")
        print(f"⏱️  Fastest Lap: {min(lap_times):.2f} s")
        print(f"⏱️  Average Lap: {np.mean(lap_times):.2f} s")
        print(f"⏱️  Total Sim Time: {step*0.01:.2f} s (Simulated in {elapsed_wall:.2f} s wall time)")
        print("="*50)
        for i, t in enumerate(lap_times, 1):
            star = " ⭐ FASTEST" if t == min(lap_times) else ""
            print(f"   Lap {i:2d}: {t:6.2f} s{star}")
        print("="*50)

if __name__ == '__main__':
    run_benchmark(10)
