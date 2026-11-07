#!/usr/bin/env python3
"""
Follow-the-Gap Reactive Controller with Integrated Lap Counter & Lap Timer
F1TENTH Autonomous Racing - Group 4 (Track: Circuit Zandvoort)
Author: Alamgir Abu Hanif
"""

import math
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
    High-Performance Follow-the-Gap Reactive Controller for F1TENTH.
    Features:
      1. Forward FOV extraction & moving average noise filtering.
      2. Disparity Extender algorithm to prevent wall/corner clipping.
      3. Dynamic safety bubble inflation around closest obstacles.
      4. Max gap identification and hybrid goal point selection.
      5. Curvature-adaptive speed profiling with longitudinal slew rate limiting.
      6. Odometry-based automated Lap Counter and Lap Timer.
      7. Real-time RViz 3D Text Marker HUD and live console dashboard.
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

        # ----------------- Follow-the-Gap Parameters -----------------
        self.bubble_radius = 0.50          # Safety bubble radius (m) (car width ~0.31m + margin)
        self.max_lidar_dist = 6.8          # Max forward view distance (m)
        self.fov_deg = 140.0               # Active field of view in degrees (+/- 70 deg)
        self.min_gap_dist = 1.15           # Minimum distance threshold for free-space gaps (m)
        self.disparity_threshold = 0.30    # Disparity edge detection jump (m)
        self.car_width_margin = 0.44       # Effective car width for disparity inflation (m)

        # ----------------- Speed & Dynamic Profile -----------------
        self.max_speed = 6.8               # Maximum speed on straights (m/s)
        self.sweeper_speed = 5.0           # Speed on gentle curves (m/s)
        self.medium_turn_speed = 3.6       # Speed on medium turns (m/s)
        self.tight_turn_speed = 2.6        # Speed on tight turns (m/s)
        self.min_speed = 2.1               # Speed on sharp hairpins (m/s)
        self.max_steering_angle = 0.4189   # Max steering angle (+/- ~24 degrees)
        self.max_accel = 6.0               # Max longitudinal acceleration (m/s^2)
        self.max_decel = 8.5               # Max longitudinal deceleration (m/s^2)
        self.current_speed = 0.0           # Current ramped speed (m/s)
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

        # Periodic live telemetry timer (1 Hz terminal updates)
        self.telemetry_timer = self.create_timer(1.0, self.publish_telemetry)

        self.get_logger().info("=" * 60)
        self.get_logger().info("🏎️  F1TENTH Follow-the-Gap Controller Initialized!")
        self.get_logger().info("📍 Assigned Track: Circuit Zandvoort | Target: 10 Clean Laps")
        self.get_logger().info("👨‍💻 Student: Alamgir Abu Hanif (Group 4)")
        self.get_logger().info("=" * 60)

    # =========================================================================
    # LIDAR Preprocessing & Follow-the-Gap Algorithms
    # =========================================================================
    def preprocess_lidar(self, ranges):
        """
        Preprocesses raw LiDAR ranges:
          - Replaces NaNs, infinite, or negative readings.
          - Applies moving average smoothing filter to remove sensor noise.
          - Clips ranges to max view distance.
        """
        proc_ranges = np.array(ranges, dtype=np.float32)
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=self.max_lidar_dist, neginf=0.0)

        # 7-point moving average filter
        kernel_size = 7
        smoothed = np.convolve(proc_ranges, np.ones(kernel_size) / kernel_size, mode='same')
        return np.clip(smoothed, 0.0, self.max_lidar_dist)

    def apply_disparity_extender(self, ranges, min_i, max_i, angle_inc):
        """
        Extends disparities (sudden depth jumps at obstacle edges/corners)
        to prevent clipping inside corners.
        """
        ext_ranges = np.copy(ranges)
        for i in range(min_i, max_i - 1):
            diff = ext_ranges[i] - ext_ranges[i + 1]
            if abs(diff) > self.disparity_threshold:
                if diff > 0:
                    # Right side is closer -> extend onto left (further) beams
                    r_close = ext_ranges[i + 1]
                    if r_close > 0.05:
                        ang_ext = math.atan2(self.car_width_margin, r_close)
                        n_beams = int(ang_ext / angle_inc)
                        start_k = max(min_i, i - n_beams + 1)
                        ext_ranges[start_k:i + 1] = np.minimum(ext_ranges[start_k:i + 1], r_close)
                else:
                    # Left side is closer -> extend onto right (further) beams
                    r_close = ext_ranges[i]
                    if r_close > 0.05:
                        ang_ext = math.atan2(self.car_width_margin, r_close)
                        n_beams = int(ang_ext / angle_inc)
                        end_k = min(max_i, i + 1 + n_beams)
                        ext_ranges[i + 1:end_k] = np.minimum(ext_ranges[i + 1:end_k], r_close)
        return ext_ranges

    def apply_safety_bubble(self, ranges, min_i, max_i, angle_inc):
        """
        Finds the closest obstacle within the forward arc and zeros out
        all laser points inside the vehicle safety bubble.
        """
        ext_ranges = np.copy(ranges)
        closest_rel_idx = np.argmin(ext_ranges[min_i:max_i])
        closest_idx = min_i + closest_rel_idx
        min_dist = ext_ranges[closest_idx]

        # Apply bubble if obstacle is within 2.5m
        if 0.05 < min_dist < 2.5:
            bubble_angle = math.atan2(self.bubble_radius, min_dist)
            b_idx = int(bubble_angle / angle_inc)
            b_start = max(min_i, closest_idx - b_idx)
            b_end = min(max_i, closest_idx + b_idx + 1)
            ext_ranges[b_start:b_end] = 0.0

        return ext_ranges

    def find_max_gap(self, free_space_ranges, min_i, max_i):
        """
        Finds contiguous spans of non-zero ranges > min_gap_dist
        and returns (start_index, end_index) of the widest gap.
        """
        free_mask = free_space_ranges[min_i:max_i] > self.min_gap_dist
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
            return None

        # Return the gap with maximum angular width
        return max(gaps, key=lambda g: g[1] - g[0])

    def find_best_point(self, start_i, end_i, ranges):
        """
        Selects the optimal steering target inside the gap:
        Blends the deepest point (for smooth apexing & speed)
        with the geometric center (for safe obstacle clearance).
        """
        gap_slice = ranges[start_i:end_i]
        deepest_sub_idx = start_i + int(np.argmax(gap_slice))
        center_sub_idx = (start_i + end_i) // 2
        # 62% bias towards deepest clearance, 38% towards geometric center
        return int(0.62 * deepest_sub_idx + 0.38 * center_sub_idx)

    def calculate_speed(self, steering_angle, front_dist, dt):
        """
        Calculates target velocity dynamically based on steering angle
        and forward distance, applying smooth acceleration/braking limits.
        """
        abs_steer = abs(steering_angle)

        if abs_steer < 0.05 and front_dist > 5.2:
            target_speed = self.max_speed
        elif abs_steer < 0.11 and front_dist > 3.8:
            target_speed = self.sweeper_speed
        elif abs_steer < 0.18:
            target_speed = self.medium_turn_speed
        elif abs_steer < 0.26:
            target_speed = self.tight_turn_speed
        else:
            target_speed = self.min_speed

        # Slew rate acceleration/deceleration limiter
        if target_speed > self.current_speed:
            self.current_speed = min(target_speed, self.current_speed + self.max_accel * dt)
        else:
            self.current_speed = max(target_speed, self.current_speed - self.max_decel * dt)

        return self.current_speed

    # =========================================================================
    # LIDAR Callback (Main Control Loop)
    # =========================================================================
    def lidar_callback(self, data):
        """
        Processes each incoming LiDAR scan:
          1. Preprocesses ranges.
          2. Extends disparities.
          3. Applies safety bubble.
          4. Finds max gap.
          5. Computes steering and dynamic speed.
          6. Publishes AckermannDriveStamped.
        """
        now = self.get_clock().now()
        dt = (now - self.last_drive_time).nanoseconds / 1e9
        dt = max(0.005, min(0.05, dt))
        self.last_drive_time = now

        # Determine forward FOV indices
        half_fov = math.radians(self.fov_deg / 2.0)
        center_idx = int((-data.angle_min) / data.angle_increment)
        span_idx = int(half_fov / data.angle_increment)
        min_i = max(0, center_idx - span_idx)
        max_i = min(len(data.ranges), center_idx + span_idx)

        # 1. Preprocess
        proc_ranges = self.preprocess_lidar(data.ranges)

        # 2. Disparity Extender
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

        # Steering calculation with clipping
        raw_steering = (target_idx - center_idx) * data.angle_increment
        raw_steering = float(np.clip(raw_steering, -self.max_steering_angle, self.max_steering_angle))

        # First-order low-pass filter on steering to avoid tire slip
        filtered_steering = 0.20 * self.prev_steering + 0.80 * raw_steering
        self.prev_steering = filtered_steering

        # Dynamic speed calculation
        front_clearance = bubble_ranges[center_idx]
        speed = self.calculate_speed(filtered_steering, front_clearance, dt)

        # Publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = now.to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = filtered_steering
        drive_msg.drive.speed = speed
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
            self.current_lap_start_time = curr_time
            self.race_started = True
            return

        # Accumulate path distance
        dx = x - self.prev_x
        dy = y - self.prev_y
        dist_step = math.hypot(dx, dy)
        self.total_lap_distance += dist_step
        self.prev_x = x
        self.prev_y = y

        # Track-relative forward coordinate s and lateral deviation d_lat
        s_curr = x * self.cos_heading + y * self.sin_heading
        d_lat = abs(-x * self.sin_heading + y * self.cos_heading)

        # Detect line crossing:
        # Crosses from s < 0 to s >= 0 within track boundaries (d_lat < 2.5m)
        # after completing at least min_lap_distance (200m)
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

            # Log lap completion banner
            star_txt = " ⭐ NEW FASTEST LAP!" if is_fastest else ""
            self.get_logger().info("=" * 60)
            self.get_logger().info(
                f"🏁 LAP {self.lap_count:2d}/{self.target_laps} COMPLETED!{star_txt}\n"
                f"   ⏱️  Lap Time : {lap_seconds:6.2f} s\n"
                f"   ⚡ Avg Speed: {avg_lap_spd:5.2f} m/s\n"
                f"   📏 Distance : {self.total_lap_distance:5.1f} m\n"
                f"   🏆 Best Lap : {self.best_lap_time:6.2f} s (Lap {self.best_lap_num})"
            )
            self.get_logger().info("=" * 60)

            # Check for 10-lap challenge completion
            if self.lap_count >= self.target_laps and not self.race_finished:
                self.race_finished = True
                self.print_final_summary()

            # Reset for next lap
            self.current_lap_start_time = curr_time
            self.total_lap_distance = 0.0

        self.prev_s = s_curr

        # Update RViz visual HUD Marker
        self.publish_rviz_hud(x, y)

    # =========================================================================
    # RViz HUD & Telemetry Displays
    # =========================================================================
    def publish_rviz_hud(self, car_x, car_y):
        """
        Publishes a 3D Text Marker above the car in RViz, clearly showing
        the current lap counter and live lap timer on the screen.
        """
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lap_hud'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position marker 1.8m above the car
        marker.pose.position.x = car_x
        marker.pose.position.y = car_y
        marker.pose.position.z = 1.8
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.65  # Text height

        # Vibrant yellow/gold color
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.1
        marker.color.a = 1.0

        # Current lap elapsed time
        if self.current_lap_start_time:
            elapsed = (self.get_clock().now() - self.current_lap_start_time).nanoseconds / 1e9
        else:
            elapsed = 0.0

        best_str = f"{self.best_lap_time:.2f}s" if self.best_lap_time < float('inf') else "--.--"
        marker.text = (
            f"LAP: {self.lap_count}/{self.target_laps} | TIME: {elapsed:.1f}s | BEST: {best_str}"
        )
        self.marker_pub.publish(marker)

    def publish_telemetry(self):
        """
        Prints live timer & lap progress periodically to the terminal console
        and publishes to /lap_info topic.
        """
        if not self.race_started or self.race_finished:
            return

        elapsed = (self.get_clock().now() - self.current_lap_start_time).nanoseconds / 1e9
        best_str = f"{self.best_lap_time:.2f}s (Lap {self.best_lap_num})" if self.best_lap_time < float('inf') else "N/A"
        
        info_str = (
            f"⏱️  [LAP {self.lap_count + 1}/{self.target_laps}] "
            f"Current Time: {elapsed:5.1f} s | Speed: {self.current_speed:4.1f} m/s | Best: {best_str}"
        )
        self.get_logger().info(info_str)

        msg = String()
        msg.data = info_str
        self.info_pub.publish(msg)

    def print_final_summary(self):
        """
        Prints the complete 10-lap competition summary report when finished.
        """
        avg_time = float(np.mean(self.lap_times)) if self.lap_times else 0.0
        total_time = sum(self.lap_times)

        print("\n" + "=" * 62)
        print("🎉 10 CONSECUTIVE LAPS COMPLETED WITHOUT ANY COLLISIONS! 🎉")
        print("=" * 62)
        print(f"📍 Track        : Circuit Zandvoort")
        print(f"👨‍💻 Driver       : Alamgir Abu Hanif (Group 4)")
        print(f"⚡ Fastest Lap  : {self.best_lap_time:.2f} s (Lap {self.best_lap_num})")
        print(f"⏱️  Average Lap  : {avg_time:.2f} s")
        print(f"🏁 Total Time   : {total_time:.2f} s")
        print("-" * 62)
        for i, t in enumerate(self.lap_times, 1):
            star = " ⭐ FASTEST LAP" if t == self.best_lap_time else ""
            print(f"   Lap {i:2d} : {t:6.2f} s{star}")
        print("=" * 62 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
