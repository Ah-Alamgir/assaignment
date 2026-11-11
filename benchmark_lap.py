import os
import sys
import math
import time
import gym
import f110_gym
import numpy as np


class OptimizedFollowTheGapController:
    def __init__(self):
        # Physical & FTG parameters
        self.bubble_radius = 0.48          # Safety bubble radius (m)
        self.max_lidar_dist = 12.0         # Gap detection forward horizon (m)
        self.fov_deg = 135.0               # Field of view (+/- 67.5 deg)
        self.min_gap_dist = 1.35           # Free space threshold (m)
        self.disparity_threshold = 0.28    # Disparity jump threshold (m)
        self.car_width_margin = 0.45       # Effective car margin for disparity (m)
        self.deepest_weight = 0.76         # Weight on deepest plateau vs gap center
        self.steer_alpha = 0.88            # Steering responsiveness factor (at dt = 0.01s)
        self.steer_gain = 1.06             # Proportional steering gain
        self.smooth_kernel = np.ones(5, dtype=np.float32) / 5.0

        # Ultra-stable 7-stage velocity profile
        self.prev_steer = 0.0
        self.v_ultra = 10.5                # Long straightaway top speed (m/s)
        self.v_max = 9.5                   # Standard straight speed (m/s)
        self.v_high = 8.0                  # Fast kink / exit speed (m/s)
        self.v_sweeper = 6.6               # Sweeping curve speed (m/s)
        self.v_medium = 5.2                # Medium corner speed (m/s)
        self.v_tight = 4.0                 # Tight chicane speed (m/s)
        self.v_min = 3.1                   # Sharp hairpin apex speed (m/s)

        self.brake_decel_cap = 4.8         # Forward braking horizon deceleration (m/s^2)
        self.brake_margin = 1.30           # Forward braking safety margin (m)

    def process_scan(self, raw_ranges, angle_min, angle_inc):
        ranges = np.asarray(raw_ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=30.0, neginf=0.0)

        half_fov = math.radians(self.fov_deg / 2.0)
        center_idx = int((-angle_min) / angle_inc)
        span_idx = int(half_fov / angle_inc)
        min_i = max(0, center_idx - span_idx)
        max_i = min(len(ranges), center_idx + span_idx)

        # 5-point moving average filter (dual horizon: 30m straightaway + 12m gap)
        smoothed = np.convolve(ranges, self.smooth_kernel, mode='same')
        long_ranges = np.clip(smoothed, 0.0, 30.0)
        proc_ranges = np.clip(smoothed, 0.0, self.max_lidar_dist)

        # Non-cascading Disparity Extender
        ext_ranges = np.copy(proc_ranges)
        diffs = np.diff(proc_ranges[min_i:max_i])
        jumps = np.flatnonzero(np.abs(diffs) > self.disparity_threshold) + min_i
        for i in jumps:
            diff = proc_ranges[i] - proc_ranges[i + 1]
            if diff > 0.0:
                r_close = float(proc_ranges[i + 1])
                if r_close > 0.05:
                    n_beams = int(math.atan2(self.car_width_margin, r_close) / angle_inc)
                    start_k = max(min_i, int(i) - n_beams + 1)
                    ext_ranges[start_k:i + 1] = np.minimum(ext_ranges[start_k:i + 1], r_close)
            else:
                r_close = float(proc_ranges[i])
                if r_close > 0.05:
                    n_beams = int(math.atan2(self.car_width_margin, r_close) / angle_inc)
                    end_k = min(max_i, int(i) + 1 + n_beams)
                    ext_ranges[i + 1:end_k] = np.minimum(ext_ranges[i + 1:end_k], r_close)

        # Safety bubble
        closest_idx = min_i + int(np.argmin(ext_ranges[min_i:max_i]))
        min_dist = float(ext_ranges[closest_idx])
        if 0.05 < min_dist < 2.5:
            bubble_angle = math.atan2(self.bubble_radius, min_dist)
            b_idx = int(bubble_angle / angle_inc)
            b_start = max(min_i, closest_idx - b_idx)
            b_end = min(max_i, closest_idx + b_idx + 1)
            ext_ranges[b_start:b_end] = 0.0

        # Vectorized max-gap finder
        free_mask = ext_ranges[min_i:max_i] > self.min_gap_dist
        if not np.any(free_mask):
            target_idx = center_idx
        else:
            padded = np.empty(len(free_mask) + 2, dtype=bool)
            padded[0] = False
            padded[1:-1] = free_mask
            padded[-1] = False
            changes = np.flatnonzero(padded[1:] != padded[:-1])
            starts = changes[0::2]
            ends = changes[1::2]
            best_k = int(np.argmax(ends - starts))
            g_start = min_i + int(starts[best_k])
            g_end = min_i + int(ends[best_k])

            gap_slice = ext_ranges[g_start:g_end]
            max_gap_val = float(np.max(gap_slice))
            deep_candidates = np.flatnonzero(gap_slice >= 0.97 * max_gap_val)
            deepest_sub_idx = g_start + int(np.mean(deep_candidates))
            center_sub_idx = (g_start + g_end) // 2
            w = self.deepest_weight
            target_idx = int(w * deepest_sub_idx + (1.0 - w) * center_sub_idx)

        raw_steer = (target_idx - center_idx) * angle_inc * self.steer_gain
        raw_steer = float(np.clip(raw_steer, -0.4189, 0.4189))

        steer = (1.0 - self.steer_alpha) * self.prev_steer + self.steer_alpha * raw_steer
        self.prev_steer = steer

        # Dual-cone lookahead speed profiling
        abs_steer = abs(steer)
        cone_half = 10
        raw_front = float(
            np.min(long_ranges[center_idx - cone_half:center_idx + cone_half + 1])
        )
        t_min = max(min_i, target_idx - 6)
        t_max = min(max_i, target_idx + 7)
        target_front = float(np.min(long_ranges[t_min:t_max]))
        effective_dist = 0.75 * raw_front + 0.25 * target_front

        if abs_steer < 0.030 and raw_front > 11.0:
            target_speed = self.v_ultra
        elif abs_steer < 0.045 and raw_front > 8.0:
            target_speed = self.v_max
        elif abs_steer < 0.075 and effective_dist > 5.5:
            target_speed = self.v_high
        elif abs_steer < 0.115 and effective_dist > 3.8:
            target_speed = self.v_sweeper
        elif abs_steer < 0.175 and effective_dist > 2.6:
            target_speed = self.v_medium
        elif abs_steer < 0.255:
            target_speed = self.v_tight
        else:
            target_speed = self.v_min

        brake_cap = math.sqrt(
            2.0 * self.brake_decel_cap * max(0.15, effective_dist - self.brake_margin)
        )
        speed_cmd = min(target_speed, max(self.v_min, brake_cap))

        return steer, speed_cmd


def run_benchmark(target_laps=10):
    sys.stdout.write("=" * 60 + "\n")
    sys.stdout.write("🏎️  F1TENTH Ultra High-Performance Controller Initialized!\n")
    sys.stdout.write("📍 Assigned Track: Circuit Zandvoort | Target: 10 Clean Laps\n")
    sys.stdout.write("👨‍💻 Student: Alamgir Abu Hanif (Group 4)\n")
    sys.stdout.write("=" * 60 + "\n\n")
    sys.stdout.flush()

    base_dir = os.path.dirname(os.path.abspath(__file__))
    map_path = os.path.join(base_dir, 'src', 'f1tenth_gym_ros', 'maps', 'Zandvoort_map')
    env = gym.make(
        'f110_gym:f110-v0',
        map=map_path,
        map_ext='.png',
        num_agents=1,
        disable_env_checker=True
    ).unwrapped

    obs, _, done, _ = env.reset(np.array([[0.0, 0.0, 1.195]]))

    controller = OptimizedFollowTheGapController()
    scan_fov = 4.7
    scan_beams = 1080
    angle_min = -scan_fov / 2.0
    angle_inc = scan_fov / scan_beams

    theta0 = 1.195
    cos_t0 = math.cos(theta0)
    sin_t0 = math.sin(theta0)

    step = 0
    prev_s = 0.0
    total_lap_dist = 0.0
    prev_x, prev_y = 0.0, 0.0
    lap_start_time = 0.0
    completed_laps = 0
    lap_times = []
    best_lap_time = float('inf')
    best_lap_num = 0

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
        total_lap_dist += math.hypot(dx, dy)
        prev_x, prev_y = x, y

        s_curr = x * cos_t0 + y * sin_t0
        d_lat = abs(-x * sin_t0 + y * cos_t0)
        curr_sim_time = step * 0.01
        elapsed = curr_sim_time - lap_start_time

        if step % 10 == 0:
            best_str = (
                f"{best_lap_time:.2f}s (Lap {best_lap_num})"
                if best_lap_time < float('inf')
                else "--.--s"
            )
            speed_kmh = speed * 3.6
            current_lap = min(completed_laps + 1, target_laps)
            info_str = (
                f"🏎️ [LAP {current_lap:2d}/{target_laps}] "
                f"⏱️ Timer: {elapsed:6.1f} s | "
                f"⚡ Speed: {speed:4.1f} m/s ({speed_kmh:4.1f} km/h) | "
                f"🏆 Best: {best_str:<17} | "
                f"📏 Lap Dist: {total_lap_dist:5.1f} m"
            )
            sys.stdout.write(f"\r{info_str}")
            sys.stdout.flush()

        if prev_s < 0.0 and s_curr >= 0.0 and d_lat < 2.5 and total_lap_dist > 200.0:
            completed_laps += 1
            lap_time = curr_sim_time - lap_start_time
            lap_times.append(lap_time)

            is_fastest = False
            if lap_time < best_lap_time:
                best_lap_time = lap_time
                best_lap_num = completed_laps
                is_fastest = True

            avg_speed = total_lap_dist / lap_time
            avg_kmh = avg_speed * 3.6
            star_txt = " ⭐ NEW FASTEST LAP!" if is_fastest else ""

            sys.stdout.write(
                "\n" + "=" * 60 + "\n"
                f"🏁 LAP {completed_laps:2d}/{target_laps} COMPLETED!{star_txt}\n"
                f"   ⏱️ Lap Time   : {lap_time:6.2f} s\n"
                f"   ⚡ Average Spd: {avg_speed:5.2f} m/s ({avg_kmh:4.1f} km/h)\n"
                f"   📏 Distance   : {total_lap_dist:5.1f} m\n"
                f"   🏆 Best Lap   : {best_lap_time:6.2f} s (Lap {best_lap_num})\n"
                + "=" * 60 + "\n\n"
            )
            sys.stdout.flush()

            lap_start_time = curr_sim_time
            total_lap_dist = 0.0

        prev_s = s_curr

    elapsed_wall = time.time() - start_wall_time
    sys.stdout.write("\n" + "=" * 60 + "\n")
    if env.collisions[0]:
        sys.stdout.write(f"❌ COLLISION OCCURRED at step {step} (sim time {step*0.01:.2f} s)!\n")
        sys.stdout.write(f"   Car position: ({x:.2f}, {y:.2f})\n")
    elif completed_laps >= target_laps:
        sys.stdout.write(f"🏆 SUCCESS: Completed {completed_laps} consecutive laps WITHOUT any collisions!\n")
        sys.stdout.write(f"⏱️  Fastest Lap: {best_lap_time:.2f} s (Lap {best_lap_num})\n")
        sys.stdout.write(f"⏱️  Average Lap: {np.mean(lap_times):.2f} s\n")
        sys.stdout.write(f"⏱️  Total Sim Time: {step*0.01:.2f} s (Simulated in {elapsed_wall:.2f} s wall time)\n")
        sys.stdout.write("=" * 60 + "\n")
        for i, t in enumerate(lap_times, 1):
            star = " ⭐ FASTEST" if t == best_lap_time else ""
            sys.stdout.write(f"   lap {i:2d}- {t:5.1f}sec |no colition{star}\n")
        sys.stdout.write("=" * 60 + "\n")
    sys.stdout.flush()


if __name__ == '__main__':
    run_benchmark(10)
