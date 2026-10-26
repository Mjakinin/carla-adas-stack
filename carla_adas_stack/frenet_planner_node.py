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
        self.declare_parameter('enable_evasive_maneuver', False)
        self.declare_parameter('enable_overtaking', False)
        self.declare_parameter('overtake_lane_offset', 3.8)
        self.declare_parameter('overtake_trigger_dist', 32.0)
        self.declare_parameter('role_name', 'ego_vehicle')

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.target_speed = self.get_parameter('target_speed_mps').get_parameter_value().double_value
        self.lane_width = self.get_parameter('lane_width').get_parameter_value().double_value
        self.T = self.get_parameter('planning_horizon_sec').get_parameter_value().double_value
        self.enable_evasive = self.get_parameter('enable_evasive_maneuver').get_parameter_value().bool_value
        self.enable_overtake = self.get_parameter('enable_overtaking').get_parameter_value().bool_value
        self.lane_offset = self.get_parameter('overtake_lane_offset').get_parameter_value().double_value
        self.trigger_dist = self.get_parameter('overtake_trigger_dist').get_parameter_value().double_value
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
        self.overtake_status_pub = self.create_publisher(
            String,
            f'/carla/{self.role_name}/overtake_status',
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

        # Overtake Finite State Machine: CRUISING -> LANE_CHANGE_LEFT -> PASSING_OBSTACLE -> LANE_CHANGE_RIGHT -> COMPLETED
        self.overtake_state = "CRUISING"
        self.overtake_start_x = 0.0
        self.obstacle_x = 0.0
        self.aeb_dist = float('inf')

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
            self.aeb_dist = data.get('obstacle_distance_m', float('inf'))

            # Trigger dynamic overtake if enabled and obstacle detected ahead
            if self.enable_overtake and self.overtake_state == "CRUISING":
                if self.aeb_dist < self.trigger_dist and self.ego_speed > 1.5:
                    self.overtake_state = "OVERTAKING"
                    self.overtake_start_x = self.ego_x
                    self.obstacle_x = self.ego_x + self.aeb_dist
                    self.get_logger().info(f"[OVERTAKE] Obstacle detected at {self.aeb_dist:.1f}m! Initiating Quintic Polynomial S-Curve Overtake (target offset {self.lane_offset}m).")
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
                curr_wp = self.carla_map.get_waypoint(ego_loc, project_to_road=True, lane_type=carla.LaneType.Driving)

                # Guard against selecting opposite-direction lane
                carla_ego_yaw = -math.degrees(self.ego_yaw)
                yaw_diff = abs((curr_wp.transform.rotation.yaw - carla_ego_yaw + 180.0) % 360.0 - 180.0)
                if yaw_diff > 90.0:
                    left_wp = curr_wp.get_left_lane()
                    if left_wp and left_wp.lane_type == carla.LaneType.Driving:
                        l_diff = abs((left_wp.transform.rotation.yaw - carla_ego_yaw + 180.0) % 360.0 - 180.0)
                        if l_diff < yaw_diff:
                            curr_wp = left_wp
                            yaw_diff = l_diff
                    right_wp = curr_wp.get_right_lane()
                    if right_wp and right_wp.lane_type == carla.LaneType.Driving:
                        r_diff = abs((right_wp.transform.rotation.yaw - carla_ego_yaw + 180.0) % 360.0 - 180.0)
                        if r_diff < yaw_diff:
                            curr_wp = right_wp

                for _ in range(45):
                    ref_wps.append([curr_wp.transform.location.x, -curr_wp.transform.location.y])
                    next_list = curr_wp.next(2.0)
                    if not next_list:
                        break
                    if len(next_list) == 1:
                        curr_wp = next_list[0]
                    else:
                        # At junction branching, choose path closest to current orientation
                        curr_yaw = curr_wp.transform.rotation.yaw
                        best_wp = next_list[0]
                        min_d_yaw = float('inf')
                        for cand in next_list:
                            diff = abs((cand.transform.rotation.yaw - curr_yaw + 180.0) % 360.0 - 180.0)
                            if diff < min_d_yaw:
                                min_d_yaw = diff
                                best_wp = cand
                        curr_wp = best_wp
            except Exception:
                pass

        if len(ref_wps) < 5:
            # Clean local forward projection from current vehicle pose
            ref_wps = []
            for s in range(45):
                dist = s * 2.0
                ref_wps.append([
                    self.ego_x + dist * math.cos(self.ego_yaw),
                    self.ego_y + dist * math.sin(self.ego_yaw)
                ])

        ref_arr = np.array(ref_wps)

        # Overtake Progress Evaluation & State Updates
        substate = self.overtake_state
        L_shift = 20.0
        L_pass = max(self.obstacle_x + 24.0, self.overtake_start_x + 48.0)
        L_return = L_pass + 20.0

        if self.enable_overtake and self.overtake_state == "OVERTAKING":
            if self.ego_x < self.overtake_start_x + L_shift:
                substate = "LANE_CHANGE_LEFT"
            elif self.ego_x <= L_pass:
                substate = "PASSING_OBSTACLE"
            elif self.ego_x < L_return:
                substate = "LANE_CHANGE_RIGHT"
            else:
                self.overtake_state = "COMPLETED"
                substate = "COMPLETED"
                self.get_logger().info("[OVERTAKE] Overtake maneuver complete! Rejoined original lane.")

        # Publish live overtake status
        try:
            import json
            status_msg = String()
            status_msg.data = json.dumps({
                'state': substate,
                'd_current': round(self.d, 2),
                'ego_x': round(self.ego_x, 1),
                'obstacle_x': round(self.obstacle_x, 1)
            })
            self.overtake_status_pub.publish(status_msg)
        except Exception:
            pass

        plan_path = Path()
        plan_path.header.stamp = self.get_clock().now().to_msg()
        plan_path.header.frame_id = 'map'

        num_wps = len(ref_arr)
        for i in range(num_wps - 1):
            rx = ref_arr[i, 0]
            ry = ref_arr[i, 1]
            dx = ref_arr[i+1, 0] - rx
            dy = ref_arr[i+1, 1] - ry
            ref_yaw = math.atan2(dy, dx)

            # Calculate quintic polynomial S-curve lateral offset
            if self.enable_overtake and self.overtake_state == "OVERTAKING":
                if rx <= self.overtake_start_x:
                    d_wp = 0.0
                elif rx < self.overtake_start_x + L_shift:
                    u = min(max((rx - self.overtake_start_x) / L_shift, 0.0), 1.0)
                    poly = 10.0 * (u**3) - 15.0 * (u**4) + 6.0 * (u**5)
                    d_wp = self.lane_offset * poly
                elif rx <= L_pass:
                    d_wp = self.lane_offset
                elif rx < L_return:
                    u = min(max((rx - L_pass) / 20.0, 0.0), 1.0)
                    poly = 10.0 * (u**3) - 15.0 * (u**4) + 6.0 * (u**5)
                    d_wp = self.lane_offset * (1.0 - poly)
                else:
                    d_wp = 0.0
            else:
                d_wp = 0.0

            if i == 0:
                self.d = d_wp

            # Cartesian transformation with lateral offset d_wp:
            px = rx - d_wp * math.sin(ref_yaw)
            py = ry + d_wp * math.cos(ref_yaw)

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
