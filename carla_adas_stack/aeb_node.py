#!/usr/bin/env python3
"""
Autonomous Emergency Braking (AEB) & Forward Collision Warning (FCW) Node
Compliant with Euro NCAP / ISO 22839 Time-to-Collision (TTC) standards.
Filters 3D LiDAR point cloud in forward driving corridor, computes TTC, and enforces safety overrides.
"""

import json
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker

import sensor_msgs_py.point_cloud2 as pc2


class AebNode(Node):
    def __init__(self):
        super().__init__('aeb_node')

        # Parameters
        self.declare_parameter('ttc_warning_threshold', 2.5)   # Seconds (Yellow FCW)
        self.declare_parameter('ttc_emergency_threshold', 1.4) # Seconds (Red AEB)
        self.declare_parameter('min_safety_distance', 2.5)     # Hard stop distance ahead of bumper (meters)
        self.declare_parameter('corridor_half_width', 1.15)    # Vehicle half-width corridor (meters)
        self.declare_parameter('corridor_max_dist', 40.0)      # Max LiDAR lookahead (meters)
        self.declare_parameter('ground_height_threshold', -1.6)# Cut off ground reflections
        self.declare_parameter('role_name', 'ego_vehicle')

        self.ttc_warn = self.get_parameter('ttc_warning_threshold').get_parameter_value().double_value
        self.ttc_crit = self.get_parameter('ttc_emergency_threshold').get_parameter_value().double_value
        self.min_dist = self.get_parameter('min_safety_distance').get_parameter_value().double_value
        self.half_width = self.get_parameter('corridor_half_width').get_parameter_value().double_value
        self.max_dist = self.get_parameter('corridor_max_dist').get_parameter_value().double_value
        self.ground_z = self.get_parameter('ground_height_threshold').get_parameter_value().double_value
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value

        # Subscribers
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            f'/carla/{self.role_name}/lidar/points',
            self.lidar_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/carla/{self.role_name}/odometry',
            self.odom_callback,
            10
        )
        self.cmd_raw_sub = self.create_subscription(
            AckermannDrive,
            f'/carla/{self.role_name}/ackermann_cmd_raw',
            self.raw_cmd_callback,
            10
        )

        # Publishers
        self.cmd_safe_pub = self.create_publisher(
            AckermannDrive,
            f'/carla/{self.role_name}/ackermann_cmd',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            f'/carla/{self.role_name}/aeb_status',
            10
        )
        self.marker_pub = self.create_publisher(
            Marker,
            f'/carla/{self.role_name}/aeb_marker',
            10
        )

        # Internal state
        self.current_speed = 0.0
        self.last_cmd = AckermannDrive()
        self.last_min_dist = None
        self.last_dist_time = None
        self.ttc = float('inf')
        self.safety_state = "NORMAL"

        # Supervised publish loop (50 Hz)
        self.timer = self.create_timer(0.02, self.supervision_loop)
        self.get_logger().info("Euro NCAP AEB / FCW Safety Node initialized.")

    def odom_callback(self, msg: Odometry):
        self.current_speed = msg.twist.twist.linear.x

    def raw_cmd_callback(self, msg: AckermannDrive):
        self.last_cmd = msg

    def lidar_callback(self, msg: PointCloud2):
        """Processes LiDAR point cloud in forward driving corridor to find closest obstacle."""
        pts_gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.fromiter(pts_gen, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

        if len(pts) == 0:
            self.ttc = float('inf')
            self.safety_state = "NORMAL"
            return

        x = pts['x']
        y = pts['y']
        z = pts['z']

        # Filter forward corridor (ignore own vehicle body x < 2.8m)
        # In lidar frame: x is forward, y is left (+/- half_width), z is height
        mask = (x > 2.8) & (x < self.max_dist) & (np.abs(y) < self.half_width) & (z > self.ground_z) & (z < 1.0)
        corridor_pts_x = x[mask]
        corridor_pts_y = y[mask]

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if len(corridor_pts_x) == 0:
            self.ttc = float('inf')
            self.safety_state = "NORMAL"
            self.last_min_dist = None
            self.publish_status(dist=float('inf'), ttc=float('inf'), state="NORMAL")
            return

        # Closest obstacle distance
        dists = np.sqrt(corridor_pts_x**2 + corridor_pts_y**2)
        min_distance = float(np.min(dists))

        # Relative velocity calculation
        v_rel = max(0.0, self.current_speed)
        if self.last_min_dist is not None and self.last_dist_time is not None:
            dt = now_sec - self.last_dist_time
            if dt > 0.01:
                measured_v_rel = (self.last_min_dist - min_distance) / dt
                # Smooth relative speed estimate with ego speed
                v_rel = max(0.0, 0.7 * measured_v_rel + 0.3 * self.current_speed)

        self.last_min_dist = min_distance
        self.last_dist_time = now_sec

        # Time-to-Collision (TTC)
        if v_rel > 0.3:
            self.ttc = min_distance / v_rel
        else:
            self.ttc = float('inf')

        # Safety state evaluation with latching / hysteresis
        old_state = self.safety_state
        if self.safety_state == "EMERGENCY_BRAKE" and self.current_speed > 0.15:
            # Remain latched in emergency brake until complete stop or obstacle removed
            if min_distance > (self.min_dist * 2.5):
                self.safety_state = "NORMAL"
            else:
                self.safety_state = "EMERGENCY_BRAKE"
        elif self.ttc <= self.ttc_crit or min_distance <= self.min_dist:
            self.safety_state = "EMERGENCY_BRAKE"
            if old_state != "EMERGENCY_BRAKE":
                self.get_logger().warn(f"AEB TRIGGERED EMERGENCY BRAKE! Dist={min_distance:.2f}m, TTC={self.ttc:.2f}s, EgoSpeed={self.current_speed*3.6:.1f}km/h")
        elif self.ttc <= self.ttc_warn or min_distance <= (self.min_dist * 1.8):
            self.safety_state = "WARNING"
        else:
            self.safety_state = "NORMAL"

        self.publish_status(dist=min_distance, ttc=self.ttc, state=self.safety_state)
        self.publish_visual_marker(min_distance, self.safety_state)

    def supervision_loop(self):
        """Passes or overrides the Ackermann drive command based on safety state."""
        safe_cmd = AckermannDrive()
        safe_cmd.steering_angle = self.last_cmd.steering_angle

        if self.safety_state == "EMERGENCY_BRAKE":
            # HARD STOP OVERRIDE
            safe_cmd.speed = 0.0
            safe_cmd.acceleration = -8.0  # Deceleration demand
        elif self.safety_state == "WARNING":
            # Decelerate / clamp speed
            safe_cmd.speed = min(self.last_cmd.speed * 0.5, 3.0)
            safe_cmd.acceleration = self.last_cmd.acceleration
        else:
            # NORMAL
            safe_cmd.speed = self.last_cmd.speed
            safe_cmd.acceleration = self.last_cmd.acceleration

        self.cmd_safe_pub.publish(safe_cmd)

    def publish_status(self, dist, ttc, state):
        status = {
            'state': state,
            'obstacle_distance_m': round(dist, 2) if dist != float('inf') else 999.0,
            'ego_speed_kmh': round(self.current_speed * 3.6, 1),
            'ttc_sec': round(ttc, 2) if ttc != float('inf') else 999.0
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def publish_visual_marker(self, dist, state):
        marker = Marker()
        marker.header.frame_id = f'{self.role_name}/base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "aeb_safety"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = min(dist, 30.0)
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5
        marker.scale.x = 0.5
        marker.scale.y = 2.0
        marker.scale.z = 1.0

        if state == "EMERGENCY_BRAKE":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.85
        elif state == "WARNING":
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.0
            marker.color.a = 0.75
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = AebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
