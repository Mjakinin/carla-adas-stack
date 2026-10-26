#!/usr/bin/env python3
"""
Extended Kalman Filter (EKF) State Estimator Node
Fuses noisy GNSS positions with high-rate IMU angular velocity and wheel speed.
Estimates vehicle pose [x, y, yaw, velocity] and covariance.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
import tf_transformations


class StateEstimatorNode(Node):
    def __init__(self):
        super().__init__('state_estimator_node')

        self.declare_parameter('role_name', 'ego_vehicle')
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            f'/carla/{self.role_name}/imu',
            self.imu_callback,
            10
        )
        self.gnss_sub = self.create_subscription(
            NavSatFix,
            f'/carla/{self.role_name}/gnss',
            self.gnss_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/carla/{self.role_name}/odometry',
            self.odom_callback,
            10
        )

        # Publisher for filtered state
        self.filtered_odom_pub = self.create_publisher(
            Odometry,
            f'/carla/{self.role_name}/filtered_odometry',
            10
        )

        # State vector: [x, y, yaw, v]
        self.x = np.zeros(4)
        # Covariance matrix P
        self.P = np.diag([1.0, 1.0, 0.1, 0.5])
        # Process noise covariance Q
        self.Q = np.diag([0.05, 0.05, 0.02, 0.1])
        # Measurement noise covariance R (GNSS)
        self.R_gnss = np.diag([2.0, 2.0])

        self.last_imu_time = None
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        self.current_yaw_rate = 0.0
        self.current_accel_x = 0.0
        self.initialized = False

        self.get_logger().info("EKF Localization State Estimator initialized.")

    def odom_callback(self, msg: Odometry):
        # Initialize EKF state from initial odometry if not set
        if not self.initialized:
            self.x[0] = msg.pose.pose.position.x
            self.x[1] = msg.pose.pose.position.y
            q = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            _, _, self.x[2] = tf_transformations.euler_from_quaternion(q)
            self.x[3] = msg.twist.twist.linear.x
            self.initialized = True
            self.get_logger().info(f"EKF initialized at position ({self.x[0]:.2f}, {self.x[1]:.2f})")

    def imu_callback(self, msg: Imu):
        """High-rate prediction step using IMU yaw rate."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_imu_time is None:
            self.last_imu_time = now
            return

        dt = now - self.last_imu_time
        self.last_imu_time = now

        if dt <= 0.0 or dt > 0.5 or not self.initialized:
            return

        self.current_yaw_rate = msg.angular_velocity.z
        self.current_accel_x = msg.linear_acceleration.x

        # Non-linear kinematic prediction:
        px, py, yaw, v = self.x
        v_pred = max(0.0, v + self.current_accel_x * dt)
        px_pred = px + v * math.cos(yaw) * dt
        py_pred = py + v * math.sin(yaw) * dt
        yaw_pred = yaw + self.current_yaw_rate * dt

        # Normalize yaw
        yaw_pred = (yaw_pred + math.pi) % (2.0 * math.pi) - math.pi

        self.x = np.array([px_pred, py_pred, yaw_pred, v_pred])

        # Jacobian F
        F = np.array([
            [1.0, 0.0, -v * math.sin(yaw) * dt, math.cos(yaw) * dt],
            [0.0, 1.0,  v * math.cos(yaw) * dt, math.sin(yaw) * dt],
            [0.0, 0.0, 1.0,                     0.0],
            [0.0, 0.0, 0.0,                     1.0]
        ])

        # Covariance propagation
        self.P = F @ self.P @ F.T + self.Q * dt

        # Publish filtered state
        self.publish_filtered_odometry()

    def gnss_callback(self, msg: NavSatFix):
        """Measurement update step when GNSS fix is received."""
        if not self.initialized:
            return

        # Reference origin (Equirectangular approximation for local tangent plane)
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude

        # Convert lat/lon difference to local UTM/Cartesian meters
        r_earth = 6371000.0
        dlat = math.radians(msg.latitude - self.origin_lat)
        dlon = math.radians(msg.longitude - self.origin_lon)
        lat_avg = math.radians((msg.latitude + self.origin_lat) / 2.0)

        # Measurement z = [x, y]
        meas_x = dlon * r_earth * math.cos(lat_avg)
        meas_y = dlat * r_earth
        z = np.array([meas_x, meas_y])

        # Measurement matrix H
        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])

        # Innovation
        y_innov = z - H @ self.x

        # Innovation covariance
        S = H @ self.P @ H.T + self.R_gnss

        # Kalman Gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # State correction
        self.x = self.x + K @ y_innov
        # Covariance update
        I_mat = np.eye(4)
        self.P = (I_mat - K @ H) @ self.P

    def publish_filtered_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = f'{self.role_name}/base_link'

        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])
        msg.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.x[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        msg.twist.twist.linear.x = float(self.x[3])
        msg.twist.twist.angular.z = float(self.current_yaw_rate)

        # Covariance mapping
        msg.pose.covariance[0] = float(self.P[0, 0])
        msg.pose.covariance[7] = float(self.P[1, 1])
        msg.pose.covariance[35] = float(self.P[2, 2])

        self.filtered_odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
