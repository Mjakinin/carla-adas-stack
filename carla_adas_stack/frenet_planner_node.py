#!/usr/bin/env python3
"""
Frenet Frame Local Trajectory Planner Node
Implements optimal quintic polynomial trajectory generation in Frenet coordinates (s, d)
based on Werling et al. (ICRA 2010) for lane keeping and dynamic obstacle avoidance.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import tf_transformations


class QuinticPolynomial:
    """Quintic polynomial for jerk-optimal lateral trajectory."""
    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([
            [T**3, T**4, T**5],
            [3*T**2, 4*T**3, 5*T**4],
            [6*T, 12*T**2, 20*T**3]
        ])
        b = np.array([
            xe - self.a0 - self.a1 * T - self.a2 * T**2,
            vxe - self.a1 - 2 * self.a2 * T,
            axe - 2 * self.a2
        ])
        try:
            x = np.linalg.solve(A, b)
            self.a3 = x[0]
            self.a4 = x[1]
            self.a5 = x[2]
        except np.linalg.LinAlgError:
            self.a3 = self.a4 = self.a5 = 0.0

    def calc_point(self, t):
        return self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

    def calc_first_derivative(self, t):
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4


class QuarticPolynomial:
    """Quartic polynomial for target velocity longitudinal profile."""
    def __init__(self, xs, vxs, axs, vxe, axe, T):
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([
            [3*T**2, 4*T**3],
            [6*T, 12*T**2]
        ])
        b = np.array([
            vxe - self.a1 - 2 * self.a2 * T,
            axe - 2 * self.a2
        ])
        try:
            x = np.linalg.solve(A, b)
            self.a3 = x[0]
            self.a4 = x[1]
        except np.linalg.LinAlgError:
            self.a3 = self.a4 = 0.0

    def calc_point(self, t):
        return self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4

    def calc_first_derivative(self, t):
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3


class FrenetPlannerNode(Node):
    def __init__(self):
        super().__init__('frenet_planner_node')

        # Parameters
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 2000)
        self.declare_parameter('target_speed_mps', 8.33)  # 30 km/h
        self.declare_parameter('lane_width', 3.5)         # Standard lane width (meters)
        self.declare_parameter('planning_horizon_sec', 3.0)
        self.declare_parameter('role_name', 'ego_vehicle')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.target_speed = self.get_parameter('target_speed_mps').get_parameter_value().double_value
        self.lane_width = self.get_parameter('lane_width').get_parameter_value().double_value
        self.T = self.get_parameter('planning_horizon_sec').get_parameter_value().double_value
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value

        # Connect to CARLA Map API for high-definition road geometry
        self.carla_map = None
        try:
            import carla
            client = carla.Client(self.host, self.port)
            client.set_timeout(5.0)
            self.carla_map = client.get_world().get_map()
            self.get_logger().info("Connected to CARLA Map for dynamic road geometry tracing.")
        except Exception as e:
            self.get_logger().warn(f"Could not connect directly to CARLA Map ({e}), falling back to parametric route.")

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/carla/{self.role_name}/odometry',
            self.odom_callback,
            10
        )
        self.aeb_status_sub = self.create_subscription(
            String,
            f'/carla/{self.role_name}/aeb_status',
            self.aeb_callback,
            10
        )

        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            f'/carla/{self.role_name}/target_path',
            10
        )
        self.cand_pub = self.create_publisher(
            MarkerArray,
            f'/carla/{self.role_name}/frenet_candidates',
            10
        )

        # Internal state
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_speed = 0.0
        self.has_odom = False

        # Frenet state: (s, s_d, s_dd, d, d_d, d_dd)
        self.s = 0.0
        self.d = 0.0
        self.current_lateral_state = 0.0  # 0.0 = center lane, +3.5 = overtake lane
        self.obstacle_detected = False

        # Plan loop at 10 Hz
        self.timer = self.create_timer(0.1, self.planning_loop)
        self.get_logger().info("Frenet Frame Trajectory Planner Node initialized.")

    def generate_town01_reference_route(self):
        """Generates a smooth reference loop for Town01."""
        # Rectangular test track with smoothed corners
        wps = []
        # Straight 1 (East)
        for x in np.linspace(100.0, 300.0, 50):
            wps.append([float(x), 0.0])
        # Turn 1
        for a in np.linspace(-math.pi/2, 0.0, 20):
            wps.append([float(300.0 + 30.0 * math.cos(a)), float(30.0 + 30.0 * math.sin(a))])
        # Straight 2 (North)
        for y in np.linspace(30.0, 200.0, 40):
            wps.append([330.0, float(y)])
        # Turn 2
        for a in np.linspace(0.0, math.pi/2, 20):
            wps.append([float(300.0 + 30.0 * math.cos(a)), float(200.0 + 30.0 * math.sin(a))])
        # Straight 3 (West)
        for x in np.linspace(300.0, 100.0, 50):
            wps.append([float(x), 230.0])
        # Turn 3
        for a in np.linspace(math.pi/2, math.pi, 20):
            wps.append([float(100.0 + 30.0 * math.cos(a)), float(200.0 + 30.0 * math.sin(a))])
        # Straight 4 (South)
        for y in np.linspace(200.0, 30.0, 40):
            wps.append([70.0, float(y)])
        # Turn 4
        for a in np.linspace(math.pi, 3*math.pi/2, 20):
            wps.append([float(100.0 + 30.0 * math.cos(a)), float(30.0 + 30.0 * math.sin(a))])
        return np.array(wps)

    def odom_callback(self, msg: Odometry):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_speed = msg.twist.twist.linear.x

        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, self.ego_yaw = tf_transformations.euler_from_quaternion(q)
        self.has_odom = True

    def aeb_callback(self, msg: String):
        try:
            import json
            data = json.loads(msg.data)
            # If warning or emergency brake, signal need to consider lateral detour
            if data['state'] in ['WARNING', 'EMERGENCY_BRAKE'] and data['obstacle_distance_m'] < 20.0:
                self.obstacle_detected = True
            elif data['obstacle_distance_m'] > 25.0:
                self.obstacle_detected = False
        except Exception:
            pass

    def planning_loop(self):
        if not self.has_odom:
            return

        import carla

        # 1. Dynamic Road Geometry Tracing
        ref_wps = []
        if self.carla_map:
            try:
                # Convert ROS ego pose (x, y) to CARLA (x, -y)
                ego_loc = carla.Location(x=float(self.ego_x), y=float(-self.ego_y), z=0.3)
                curr_wp = self.carla_map.get_waypoint(ego_loc)
                for _ in range(40):
                    ref_wps.append([curr_wp.transform.location.x, -curr_wp.transform.location.y])
                    next_list = curr_wp.next(2.0)
                    if not next_list:
                        break
                    curr_wp = next_list[0]
            except Exception:
                pass

        if len(ref_wps) < 5:
            # Fallback to parametric route if map query unavailable
            ref_wps = self.generate_town01_reference_route()

        ref_arr = np.array(ref_wps)

        # Decide lateral target: If obstacle detected in current lane, initiate overtake offset
        if self.obstacle_detected:
            target_d = self.lane_width  # Shift 3.5m left
        else:
            target_d = 0.0             # Lane center

        # Generate Quintic Lateral Trajectory
        d_poly = QuinticPolynomial(
            xs=self.d, vxs=0.0, axs=0.0,
            xe=target_d, vxe=0.0, axe=0.0,
            T=self.T
        )

        # Generate Longitudinal Profile
        s_poly = QuarticPolynomial(
            xs=0.0, vxs=self.ego_speed, axs=0.0,
            vxe=self.target_speed, axe=0.0,
            T=self.T
        )

        # Sample trajectory points
        dt = 0.1
        steps = int(self.T / dt)
        plan_path = Path()
        plan_path.header.stamp = self.get_clock().now().to_msg()
        plan_path.header.frame_id = 'map'

        num_wps = len(ref_arr)

        for i in range(steps):
            t = i * dt
            s_val = s_poly.calc_point(t)
            d_val = d_poly.calc_point(t)

            # Map s_val to index in reference route
            wp_idx = min(int(s_val * 0.5), num_wps - 2)
            next_idx = wp_idx + 1

            rx = ref_arr[wp_idx, 0]
            ry = ref_arr[wp_idx, 1]
            dx = ref_arr[next_idx, 0] - rx
            dy = ref_arr[next_idx, 1] - ry
            ref_yaw = math.atan2(dy, dx)

            # Cartesian transformation from Frenet (s, d):
            px = rx - d_val * math.sin(ref_yaw)
            py = ry + d_val * math.cos(ref_yaw)

            pose = PoseStamped()
            pose.header = plan_path.header
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.position.z = 0.0

            q = tf_transformations.quaternion_from_euler(0.0, 0.0, ref_yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            plan_path.poses.append(pose)

        # Update current d for next cycle
        self.d = d_poly.calc_point(0.1)
        self.path_pub.publish(plan_path)


def main(args=None):
    rclpy.init(args=args)
    node = FrenetPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
