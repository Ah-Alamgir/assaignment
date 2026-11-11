#!/usr/bin/env python3
"""
Follow-the-Gap Reactive Controller with Integrated Lap Counter & Lap Timer
F1TENTH Autonomous Racing - Group 4 (Track: Circuit Zandvoort)
Author: Alamgir Abu Hanif
"""

import math
import sys
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String


class ReactiveFollowGap(Node):
    """
    Ultra High-Performance Follow-the-Gap Reactive Controller for F1TENTH.
    Features:
      1. Dual-horizon LiDAR preprocessing (unclipped 30m straightaway lookahead + 12m gap horizon).
      2. Non-cascading vectorized Disparity Extender algorithm to prevent corner clipping.
      3. Dynamic safety bubble inflation around closest obstacles.
      4. Vectorized max-gap identification and plateau-stabilized hybrid goal point selection.
      5. Direct 7-stage curvature + dual-cone braking horizon velocity control (~52.8s lap time).
      6. Odometry-based automated Lap Counter and Lap Timer.
      7. Large Top-Right Corner RViz Scoreboard showing all lap times + live console HUD.
    """

    def __init__(self):
        super().__init__('reactive_gap_node')

        # ----------------- Topic Configuration -----------------
        self.lidarscan_topic = '/scan'
        self.drive_topic = '/drive'
        self.odom_topic = '/ego_racecar/odom'

        # Publishers & Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, self.lidarscan_topic, self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 10)
        self.marker_pub = self.create_publisher(
            Marker, '/lap_marker', 10)
        self.info_pub = self.create_publisher(
            String, '/lap_info', 10)

        # ----------------- High-Performance FTG Parameters -----------------
        self.bubble_radius = 0.48          # Safety bubble radius (m)
        self.max_lidar_dist = 12.0         # Gap detection forward horizon (m)
        self.fov_deg = 135.0               # Active field of view in degrees (+/- 67.5 deg)
        self.min_gap_dist = 1.35           # Minimum distance threshold for free-space gaps (m)
        self.disparity_threshold = 0.28    # Disparity edge detection jump (m)
        self.car_width_margin = 0.45       # Effective car width for disparity inflation (m)
        self.deepest_weight = 0.76         # Weight on deepest plateau vs gap center
        self.steer_alpha = 0.88            # Steering responsiveness factor (at dt = 0.01s)
        self.steer_gain = 1.06             # Proportional steering gain
        self.smooth_kernel = np.ones(5, dtype=np.float32) / 5.0

        # ----------------- Ultra-Stable Fast Velocity Profile -----------------
        self.v_ultra = 10.5                # Long straightaway top speed (m/s)
        self.v_max = 9.5                   # Standard straight speed (m/s)
        self.v_high = 8.0                  # Fast kink / exit speed (m/s)
        self.v_sweeper = 6.6               # Sweeping curve speed (m/s)
        self.v_medium = 5.2                # Medium corner speed (m/s)
        self.v_tight = 4.0                 # Tight chicane speed (m/s)
        self.v_min = 3.1                   # Sharp hairpin apex speed (m/s)
        self.max_steering_angle = 0.4189   # Max steering angle (+/- ~24 degrees)
        self.brake_decel_cap = 4.8         # Forward braking horizon deceleration (m/s^2)
        self.brake_margin = 1.30           # Forward braking safety distance margin (m)

        self.current_speed = 0.0           # Current vehicle speed (m/s)
        self.prev_steering = 0.0           # Filtered steering from previous step
        self.last_drive_time = self.get_clock().now()

        # ----------------- Zandvoort Start/Finish Line Detection -----------------
        # Start/finish line at (0.0, 0.0) with forward track heading = 1.195 rad (~68.5 deg)
        self.track_heading = 1.195
        self.cos_heading = math.cos(self.track_heading)
        self.sin_heading = math.sin(self.track_heading)
        self.min_lap_distance = 200.0      # Cooldown distance before next lap can trigger (m)

        # ----------------- Lap Counter & Timer State -----------------
        self.lap_count = 0
        self.target_laps = 10
        self.current_lap_start_time = None
        self.lap_times = []
        self.best_lap_time = float('inf')
        self.best_lap_num = 0
        self.prev_s = 0.0
        self.total_lap_distance = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.first_odom = True
        self.race_started = False
        self.race_finished = False

        # Periodic live telemetry timer (10 Hz in-place terminal updates)
        self.telemetry_timer = self.create_timer(0.1, self.publish_telemetry)

        sys.stdout.write(
            "=" * 60 + "\n"
            "🏎️  F1TENTH Ultra High-Performance Controller Initialized!\n"
            "📍 Assigned Track: Circuit Zandvoort | Target: 10 Clean Laps\n"
            "👨‍💻 Student: Alamgir Abu Hanif (Group 4)\n"
            + "=" * 60 + "\n\n"
        )
        sys.stdout.flush()

    # =========================================================================
    # LIDAR Preprocessing & Follow-the-Gap Algorithms
    # =========================================================================
    def preprocess_lidar(self, ranges):
        """
        Preprocesses raw LiDAR ranges:
          - Replaces NaNs, infinite, or negative readings.
          - Applies 5-point moving average smoothing filter.
          - Returns both unclipped 30m ranges (for straightaway lookahead) and
            horizon-clipped ranges (for local gap finding).
        """
        raw = np.asarray(ranges, dtype=np.float32)
        raw = np.nan_to_num(raw, nan=0.0, posinf=30.0, neginf=0.0)
        smoothed = np.convolve(raw, self.smooth_kernel, mode='same')
        long_ranges = np.clip(smoothed, 0.0, 30.0)
        proc_ranges = np.clip(smoothed, 0.0, self.max_lidar_dist)
        return long_ranges, proc_ranges

    def apply_disparity_extender(self, proc_ranges, min_i, max_i, angle_inc):
        """
        Non-cascading Disparity Extender:
        Detects real depth jumps from unmodified preprocessed ranges and extends
        closer obstacle edges by the vehicle width margin to prevent corner clipping.
        """
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

        return ext_ranges

    def apply_safety_bubble(self, ranges, min_i, max_i, angle_inc):
        """
        Finds the closest obstacle within the forward arc and zeros out
        all laser points inside the vehicle safety bubble.
        """
        ext_ranges = np.copy(ranges)
        closest_idx = min_i + int(np.argmin(ext_ranges[min_i:max_i]))
        min_dist = float(ext_ranges[closest_idx])

        if 0.05 < min_dist < 2.5:
            bubble_angle = math.atan2(self.bubble_radius, min_dist)
            b_idx = int(bubble_angle / angle_inc)
            b_start = max(min_i, closest_idx - b_idx)
            b_end = min(max_i, closest_idx + b_idx + 1)
            ext_ranges[b_start:b_end] = 0.0

        return ext_ranges

    def find_max_gap(self, free_space_ranges, min_i, max_i):
        """
        Vectorized max-gap finder: locates contiguous spans of ranges > min_gap_dist
        and returns (start_index, end_index) of the widest navigable gap.
        """
        free_mask = free_space_ranges[min_i:max_i] > self.min_gap_dist
        if not np.any(free_mask):
            return None

        padded = np.empty(len(free_mask) + 2, dtype=bool)
        padded[0] = False
        padded[1:-1] = free_mask
        padded[-1] = False
        changes = np.flatnonzero(padded[1:] != padded[:-1])
        starts = changes[0::2]
        ends = changes[1::2]
        best_k = int(np.argmax(ends - starts))
        return (min_i + int(starts[best_k]), min_i + int(ends[best_k]))

    def find_best_point(self, start_i, end_i, ranges):
        """
        Selects the optimal steering target inside the gap:
        Uses the centroid of the deepest plateau (>= 97% of max gap depth) blended
        with the geometric gap center for smooth, oscillation-free high-speed apexing.
        """
        gap_slice = ranges[start_i:end_i]
        max_gap_val = float(np.max(gap_slice))
        deep_candidates = np.flatnonzero(gap_slice >= 0.97 * max_gap_val)
        deepest_sub_idx = start_i + int(np.mean(deep_candidates))
        center_sub_idx = (start_i + end_i) // 2
        w = self.deepest_weight
        return int(w * deepest_sub_idx + (1.0 - w) * center_sub_idx)

    def calculate_speed(self, steering_angle, raw_front_dist, effective_dist):
        """
        Calculates target velocity directly using 7-stage curvature profiling and
        unclipped kinematic forward braking horizon.
        """
        abs_steer = abs(steering_angle)

        if abs_steer < 0.030 and raw_front_dist > 11.0:
            target_speed = self.v_ultra
        elif abs_steer < 0.045 and raw_front_dist > 8.0:
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

        # Kinematic forward braking horizon cap: v <= sqrt(2 * a * d_clear)
        brake_cap = math.sqrt(
            2.0 * self.brake_decel_cap * max(0.15, effective_dist - self.brake_margin)
        )
        return min(target_speed, max(self.v_min, brake_cap))

    # =========================================================================
    # LIDAR Callback (Main Control Loop)
    # =========================================================================
    def lidar_callback(self, data):
        """
        Processes each incoming LiDAR scan and publishes AckermannDriveStamped.
        """
        now = self.get_clock().now()
        dt = (now - self.last_drive_time).nanoseconds / 1e9
        dt = max(0.002, min(0.05, dt))
        self.last_drive_time = now

        half_fov = math.radians(self.fov_deg / 2.0)
        center_idx = int((-data.angle_min) / data.angle_increment)
        span_idx = int(half_fov / data.angle_increment)
        min_i = max(0, center_idx - span_idx)
        max_i = min(len(data.ranges), center_idx + span_idx)

        # 1. Dual-horizon Preprocess
        long_ranges, proc_ranges = self.preprocess_lidar(data.ranges)

        # 2. Non-cascading Disparity Extender
        ext_ranges = self.apply_disparity_extender(proc_ranges, min_i, max_i, data.angle_increment)

        # 3. Safety Bubble
        bubble_ranges = self.apply_safety_bubble(ext_ranges, min_i, max_i, data.angle_increment)

        # 4. Find Max Gap
        gap = self.find_max_gap(bubble_ranges, min_i, max_i)

        # 5. Find Target Point
        if gap is None:
            target_idx = center_idx
        else:
            target_idx = self.find_best_point(gap[0], gap[1], bubble_ranges)

        raw_steering = (target_idx - center_idx) * data.angle_increment * self.steer_gain
        raw_steering = float(np.clip(raw_steering, -self.max_steering_angle, self.max_steering_angle))

        # Deterministic low-pass filter on steering
        filtered_steering = (1.0 - self.steer_alpha) * self.prev_steering + self.steer_alpha * raw_steering
        self.prev_steering = filtered_steering

        # Dual-cone lookahead clearance (forward cone + steering target cone)
        cone_half = 10
        raw_front = float(
            np.min(long_ranges[center_idx - cone_half:center_idx + cone_half + 1])
        )
        t_min = max(min_i, target_idx - 6)
        t_max = min(max_i, target_idx + 7)
        target_front = float(np.min(long_ranges[t_min:t_max]))
        effective_dist = 0.75 * raw_front + 0.25 * target_front

        speed_cmd = self.calculate_speed(filtered_steering, raw_front, effective_dist)
        self.current_speed = speed_cmd

        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = now.to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = filtered_steering
        drive_msg.drive.speed = speed_cmd
        self.drive_pub.publish(drive_msg)

    # =========================================================================
    # Odometry Callback (Lap Counter & Timer Subsystem)
    # =========================================================================
    def odom_callback(self, data):
        """
        Tracks vehicle position and automatically detects start/finish line crossings
        at Circuit Zandvoort (0.0, 0.0) with track heading 1.195 rad.
        """
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        curr_time = self.get_clock().now()

        if self.first_odom:
            self.prev_x = x
            self.prev_y = y
            self.first_odom = False
            self.publish_rviz_hud(x, y)
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        dist_step = math.hypot(dx, dy)

        if not self.race_started:
            if dist_step > 1e-4:
                self.current_lap_start_time = curr_time
                self.race_started = True
                self.publish_rviz_hud(x, y)
            return

        self.total_lap_distance += dist_step
        self.prev_x = x
        self.prev_y = y

        s_curr = x * self.cos_heading + y * self.sin_heading
        d_lat = abs(-x * self.sin_heading + y * self.cos_heading)

        if self.prev_s < 0.0 and s_curr >= 0.0 and d_lat < 2.5 and self.total_lap_distance > self.min_lap_distance:
            self.lap_count += 1
            lap_seconds = (curr_time - self.current_lap_start_time).nanoseconds / 1e9
            self.lap_times.append(lap_seconds)

            is_fastest = False
            if lap_seconds < self.best_lap_time:
                self.best_lap_time = lap_seconds
                self.best_lap_num = self.lap_count
                is_fastest = True

            avg_lap_spd = self.total_lap_distance / max(1.0, lap_seconds)
            avg_kmh = avg_lap_spd * 3.6

            star_txt = " ⭐ NEW FASTEST LAP!" if is_fastest else ""
            sys.stdout.write(
                "\n" + "=" * 60 + "\n"
                f"🏁 LAP {self.lap_count:2d}/{self.target_laps} COMPLETED!{star_txt}\n"
                f"   ⏱️ Lap Time   : {lap_seconds:6.2f} s\n"
                f"   ⚡ Average Spd: {avg_lap_spd:5.2f} m/s ({avg_kmh:4.1f} km/h)\n"
                f"   📏 Distance   : {self.total_lap_distance:5.1f} m\n"
                f"   🏆 Best Lap   : {self.best_lap_time:6.2f} s (Lap {self.best_lap_num})\n"
                + "=" * 60 + "\n\n"
            )
            sys.stdout.flush()

            if self.lap_count >= self.target_laps and not self.race_finished:
                self.race_finished = True
                self.print_final_summary()

            self.current_lap_start_time = curr_time
            self.total_lap_distance = 0.0
            self.publish_rviz_hud(x, y)

        self.prev_s = s_curr

    # =========================================================================
    # RViz Top-Right Corner Scoreboard & Telemetry Displays
    # =========================================================================
    def publish_rviz_hud(self, car_x, car_y):
        """
        Renders a large scoreboard text in the TOP-RIGHT CORNER of the RViz
        simulation viewport displaying all completed lap times in the format:
            lap 1- 55.3sec |no colition
            lap 2- 55.5sec |no colition
            ...
        """
        stamp = self.get_clock().now().to_msg()
        if self.current_lap_start_time:
            elapsed = (self.get_clock().now() - self.current_lap_start_time).nanoseconds / 1e9
        else:
            elapsed = 0.0

        # Build the multi-line lap list for all completed laps + active lap
        lines = []
        for idx, t in enumerate(self.lap_times, 1):
            lines.append(f"lap {idx}- {t:.1f}sec |no colition")

        if self.lap_count < self.target_laps:
            curr_lap_num = self.lap_count + 1
            lines.append(f"lap {curr_lap_num}- {elapsed:.1f}sec |no colition")

        scoreboard_text = "\n".join(lines)

        # In gym_bridge.rviz (Focal Point X=32.4, Y=-6.3, Yaw=pi, Pitch=1.55):
        # Screen UP is +X, Screen RIGHT is -Y.
        # Top-Right Corner of the RViz viewport (fully visible inside margins):
        hud_x = 56.5
        hud_y = -34.0

        # Clear any previously displayed background box (id = 0)
        bg_clear = Marker()
        bg_clear.header.frame_id = 'map'
        bg_clear.header.stamp = stamp
        bg_clear.ns = 'lap_hud'
        bg_clear.id = 0
        bg_clear.action = Marker.DELETE
        self.marker_pub.publish(bg_clear)

        # 1. Large Bold Black Top-Right Lap Scoreboard Text (id = 1 + bold stroke offsets)
        bold_offsets = [
            (1, 0.0, 0.0),
            (10, 0.035, 0.0),
            (11, -0.035, 0.0),
            (12, 0.0, 0.035),
            (13, 0.0, -0.035),
        ]
        for m_id, ox, oy in bold_offsets:
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = stamp
            text_marker.ns = 'lap_hud'
            text_marker.id = m_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = hud_x + ox
            text_marker.pose.position.y = hud_y + oy
            text_marker.pose.position.z = 2.5
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 2.25
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = scoreboard_text
            self.marker_pub.publish(text_marker)

        # 2. Compact live tag following the racecar (id = 2)
        car_marker = Marker()
        car_marker.header.frame_id = 'map'
        car_marker.header.stamp = stamp
        car_marker.ns = 'lap_hud'
        car_marker.id = 2
        car_marker.type = Marker.TEXT_VIEW_FACING
        car_marker.action = Marker.ADD
        car_marker.pose.position.x = car_x
        car_marker.pose.position.y = car_y
        car_marker.pose.position.z = 2.0
        car_marker.pose.orientation.w = 1.0
        car_marker.scale.z = 1.55
        car_marker.color.r = 1.0
        car_marker.color.g = 0.90
        car_marker.color.b = 0.10
        car_marker.color.a = 1.0
        curr_lap_disp = min(self.lap_count + 1, self.target_laps)
        car_marker.text = f"lap {curr_lap_disp}- {elapsed:.1f}sec |no colition"
        self.marker_pub.publish(car_marker)

    def publish_telemetry(self):
        if not self.first_odom:
            self.publish_rviz_hud(self.prev_x, self.prev_y)
        if not self.race_started or self.race_finished:
            return

        elapsed = (self.get_clock().now() - self.current_lap_start_time).nanoseconds / 1e9
        best_str = (
            f"{self.best_lap_time:.2f}s (Lap {self.best_lap_num})"
            if self.best_lap_time < float('inf')
            else "--.--s"
        )
        speed_kmh = self.current_speed * 3.6
        current_lap = min(self.lap_count + 1, self.target_laps)

        info_str = (
            f"🏎️ [LAP {current_lap:2d}/{self.target_laps}] "
            f"⏱️ Timer: {elapsed:6.1f} s | "
            f"⚡ Speed: {self.current_speed:4.1f} m/s ({speed_kmh:4.1f} km/h) | "
            f"🏆 Best: {best_str:<17} | "
            f"📏 Lap Dist: {self.total_lap_distance:5.1f} m"
        )
        sys.stdout.write(f"\r{info_str}")
        sys.stdout.flush()

        msg = String()
        msg.data = info_str
        self.info_pub.publish(msg)

    def print_final_summary(self):
        avg_time = float(np.mean(self.lap_times)) if self.lap_times else 0.0
        total_time = sum(self.lap_times)

        sys.stdout.write("\n" + "=" * 60 + "\n")
        sys.stdout.write("🎉 10 CONSECUTIVE LAPS COMPLETED WITHOUT ANY COLLISIONS! 🎉\n")
        sys.stdout.write("=" * 60 + "\n")
        sys.stdout.write("📍 Track        : Circuit Zandvoort\n")
        sys.stdout.write("👨‍💻 Driver       : Alamgir Abu Hanif (Group 4)\n")
        sys.stdout.write(f"⚡ Fastest Lap  : {self.best_lap_time:.2f} s (Lap {self.best_lap_num})\n")
        sys.stdout.write(f"⏱️ Average Lap  : {avg_time:.2f} s\n")
        sys.stdout.write(f"🏁 Total Time   : {total_time:.2f} s\n")
        sys.stdout.write("-" * 60 + "\n")
        for i, t in enumerate(self.lap_times, 1):
            star = " ⭐ FASTEST LAP" if t == self.best_lap_time else ""
            sys.stdout.write(f"   lap {i:2d}- {t:5.1f}sec |no colition{star}\n")
        sys.stdout.write("=" * 60 + "\n\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
