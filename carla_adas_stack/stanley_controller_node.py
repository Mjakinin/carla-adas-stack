#!/usr/bin/env python3
"""
Stanley Path Tracking Controller Node
Implements the Stanley lateral steering control law and curvature-aware longitudinal speed regulation.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker, MarkerArray

import tf_transformations


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class StanleyControllerNode(Node):
    def __init__(self):
        super().__init__('stanley_controller_node')

        # Parameters
        self.declare_parameter('k_stanley', 0.8)       # Cross-track gain
        self.declare_parameter('soft_speed', 1.0)      # Regularization term epsilon (m/s)
        self.declare_parameter('wheelbase', 2.87)      # Wheelbase of Tesla Model 3 (meters)
        self.declare_parameter('max_steer_rad', 0.65)  # Max steering angle (~37 deg)
        self.declare_parameter('target_speed_mps', 8.33)  # Target cruising speed (30 km/h)
        self.declare_parameter('min_speed_mps', 3.5)   # Minimum speed in tight curves
        self.declare_parameter('role_name', 'ego_vehicle')

        self.k_stanley = self.get_parameter('k_stanley').get_parameter_value().double_value
        self.soft_speed = self.get_parameter('soft_speed').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_steer_rad = self.get_parameter('max_steer_rad').get_parameter_value().double_value
        self.target_speed_mps = self.get_parameter('target_speed_mps').get_parameter_value().double_value
        self.min_speed_mps = self.get_parameter('min_speed_mps').get_parameter_value().double_value
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/carla/{self.role_name}/odometry',
            self.odom_callback,
            10
        )
        self.path_sub = self.create_subscription(
            Path,
            f'/carla/{self.role_name}/target_path',
            self.path_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(
            AckermannDrive,
            f'/carla/{self.role_name}/ackermann_cmd',
            10
        )
        self.debug_pub = self.create_publisher(
            MarkerArray,
            f'/carla/{self.role_name}/stanley_debug',
            10
        )

        # State
        self.current_pose = None
        self.current_yaw = 0.0
        self.current_speed = 0.0
        self.waypoints = []  # List of [x, y, yaw]
        self.path_received = False

        # Periodic control loop (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Stanley Controller Node initialized.")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(q)
        self.current_speed = msg.twist.twist.linear.x

    def path_callback(self, msg: Path):
        """Update reference path from Path message."""
        pts = []
        for i in range(len(msg.poses)):
            x = msg.poses[i].pose.position.x
            y = msg.poses[i].pose.position.y
            pts.append([x, y])

        if len(pts) >= 2:
            self.waypoints = self.compute_headings(pts)
            self.path_received = True

    def compute_headings(self, pts):
        """Calculate tangent heading for each waypoint."""
        wps = []
        num = len(pts)
        for i in range(num):
            if i < num - 1:
                dx = pts[i+1][0] - pts[i][0]
                dy = pts[i+1][1] - pts[i][1]
            else:
                dx = pts[i][0] - pts[i-1][0]
                dy = pts[i][1] - pts[i-1][1]
            yaw = math.atan2(dy, dx)
            wps.append([pts[i][0], pts[i][1], yaw])
        return wps

    def find_nearest_waypoint(self, fx, fy):
        """Finds the index of the closest waypoint to front axle (fx, fy)."""
        min_dist = float('inf')
        nearest_idx = 0
        for i, wp in enumerate(self.waypoints):
            dist = math.hypot(wp[0] - fx, wp[1] - fy)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        return nearest_idx, min_dist

    def control_loop(self):
        if self.current_pose is None or len(self.waypoints) < 2:
            return

        # 1. Calculate Front Axle Position
        rx = self.current_pose.position.x
        ry = self.current_pose.position.y
        fx = rx + (self.wheelbase / 2.0) * math.cos(self.current_yaw)
        fy = ry + (self.wheelbase / 2.0) * math.sin(self.current_yaw)

        # 2. Nearest Waypoint
        idx, _ = self.find_nearest_waypoint(fx, fy)
        nearest_wp = self.waypoints[idx]

        # 3. Heading Error psi_e (look ahead 1 waypoint along path to anticipate curves)
        lookahead_idx = min(idx + 1, len(self.waypoints) - 1)
        wp_yaw = self.waypoints[lookahead_idx][2]
        heading_error = normalize_angle(wp_yaw - self.current_yaw)

        # 4. Cross-Track Error e_ct (measured at front axle)
        dx = fx - nearest_wp[0]
        dy = fy - nearest_wp[1]
        cross_track_error = dx * math.sin(nearest_wp[2]) - dy * math.cos(nearest_wp[2])

        # 5. Stanley Lateral Control Law
        # delta = psi_e + arctan( (k * e_ct) / (v + soft_speed) )
        effective_speed = max(0.5, abs(self.current_speed)) + self.soft_speed
        lateral_correction = math.atan2(self.k_stanley * cross_track_error, effective_speed)
        steer_cmd = heading_error + lateral_correction

        # Saturate steering
        steer_cmd = float(np.clip(steer_cmd, -self.max_steer_rad, self.max_steer_rad))

        # Periodic debug logging (2 Hz)
        if not hasattr(self, 'last_log_t') or (self.get_clock().now().nanoseconds - self.last_log_t) > 5e8:
            self.last_log_t = self.get_clock().now().nanoseconds
            self.get_logger().info(f"[STANLEY] v={self.current_speed*3.6:.1f}km/h, heading_err={math.degrees(heading_error):.1f}°, ect={cross_track_error:.2f}m, steer={math.degrees(steer_cmd):.1f}°")

        # 6. Adaptive Longitudinal Speed (Curve slowdown)
        steer_ratio = abs(steer_cmd) / self.max_steer_rad
        desired_speed = self.target_speed_mps * (1.0 - 0.6 * steer_ratio)
        desired_speed = max(self.min_speed_mps, desired_speed)

        # 7. Publish Ackermann Command
        cmd = AckermannDrive()
        cmd.speed = float(desired_speed)
        cmd.steering_angle = float(steer_cmd)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StanleyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
