#!/usr/bin/env python3
"""
CARLA ROS 2 Interface Node
Spawns the Ego Vehicle and sensors (Camera, LiDAR, IMU, GNSS) in CARLA Simulator.
Bridges telemetry to standard ROS 2 topics and accepts Ackermann drive commands.
"""

import math
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, PointField, Imu, NavSatFix, NavSatStatus, CameraInfo
from ackermann_msgs.msg import AckermannDrive

import tf2_ros
import tf_transformations

try:
    import carla
except ImportError:
    print("ERROR: Could not import 'carla' Python module.", file=sys.stderr)
    sys.exit(1)


class CarlaInterfaceNode(Node):
    def __init__(self):
        super().__init__('carla_interface_node')

        # Declare parameters
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 2000)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('vehicle_filter', 'vehicle.tesla.model3')
        self.declare_parameter('role_name', 'ego_vehicle')
        self.declare_parameter('spawn_index', 0)
        self.declare_parameter('max_steer_angle', 0.70)  # ~40 degrees in radians
        self.declare_parameter('kp_speed', 0.25)
        self.declare_parameter('ki_speed', 0.05)
        self.declare_parameter('kd_speed', 0.01)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.vehicle_filter = self.get_parameter('vehicle_filter').get_parameter_value().string_value
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value
        self.spawn_index = self.get_parameter('spawn_index').get_parameter_value().integer_value
        self.max_steer_angle = self.get_parameter('max_steer_angle').get_parameter_value().double_value
        self.kp_speed = self.get_parameter('kp_speed').get_parameter_value().double_value
        self.ki_speed = self.get_parameter('ki_speed').get_parameter_value().double_value
        self.kd_speed = self.get_parameter('kd_speed').get_parameter_value().double_value

        self.get_logger().info(f"Connecting to CARLA Server at {self.host}:{self.port}...")

        self.client = None
        self.world = None
        self.ego_vehicle = None
        self.sensors = []

        # Connect to CARLA
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(self.timeout)
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            self.get_logger().info(f"Connected to CARLA Map: '{self.map.name}'")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            raise e

        # Initialize Publishers
        self.odom_pub = self.create_publisher(Odometry, f'/carla/{self.role_name}/odometry', 10)
        self.image_pub = self.create_publisher(Image, f'/carla/{self.role_name}/camera/image_raw', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, f'/carla/{self.role_name}/camera/camera_info', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, f'/carla/{self.role_name}/lidar/points', 10)
        self.gnss_pub = self.create_publisher(NavSatFix, f'/carla/{self.role_name}/gnss', 10)
        self.imu_pub = self.create_publisher(Imu, f'/carla/{self.role_name}/imu', 10)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_sensor_tfs()

        # Control Subscriber
        self.ackermann_sub = self.create_subscription(
            AckermannDrive,
            f'/carla/{self.role_name}/ackermann_cmd',
            self.ackermann_callback,
            10
        )

        # Target control states
        self.target_speed = 0.0
        self.target_steer = 0.0
        self.current_speed = 0.0
        self.speed_integral = 0.0
        self.speed_prev_error = 0.0
        self.emergency_brake_active = False

        self.shutting_down = False

        # Spawn Vehicle & Sensors
        self.spawn_ego_vehicle()
        self.spawn_sensors()

        # Timer for telemetry & control loop (50 Hz)
        self.timer = self.create_timer(0.02, self.update_telemetry_and_control)
        self.get_logger().info("CARLA ROS 2 Interface Node initialized successfully!")

    def spawn_ego_vehicle(self):
        """Spawns the ego vehicle at a designated spawn point in the CARLA world."""
        # Clean up existing actors with matching role_name
        for actor in self.world.get_actors().filter('vehicle.*'):
            if actor.attributes.get('role_name') == self.role_name:
                self.get_logger().info(f"Cleaning up previous ego vehicle actor {actor.id}")
                actor.destroy()

        blueprint_library = self.world.get_blueprint_library()
        bp = blueprint_library.find(self.vehicle_filter)
        if not bp:
            self.get_logger().warn(f"Blueprint '{self.vehicle_filter}' not found, falling back to standard car.")
            bp = blueprint_library.filter('vehicle.*')[0]

        bp.set_attribute('role_name', self.role_name)
        if bp.has_attribute('color'):
            color = bp.get_attribute('color').recommended_values[0]
            bp.set_attribute('color', color)

        spawn_points = self.map.get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found in current CARLA map!")

        spawn_point = spawn_points[self.spawn_index % len(spawn_points)]
        self.ego_vehicle = self.world.spawn_actor(bp, spawn_point)
        self.get_logger().info(f"Spawned ego vehicle '{bp.id}' at location: {spawn_point.location}")

    def spawn_sensors(self):
        """Spawns RGB camera, LiDAR, GNSS, and IMU on the ego vehicle."""
        blueprint_library = self.world.get_blueprint_library()

        # 1. Front RGB Camera
        cam_bp = blueprint_library.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '800')
        cam_bp.set_attribute('image_size_y', '600')
        cam_bp.set_attribute('fov', '90.0')
        cam_transform = carla.Transform(carla.Location(x=1.8, y=0.0, z=1.6), carla.Rotation(pitch=-3.0))
        camera = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.ego_vehicle)
        camera.listen(self.on_camera_image)
        self.sensors.append(camera)

        # 2. 3D LiDAR Sensor
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('range', '60.0')
        lidar_bp.set_attribute('points_per_second', '100000')
        lidar_bp.set_attribute('rotation_frequency', '20.0')
        lidar_bp.set_attribute('upper_fov', '10.0')
        lidar_bp.set_attribute('lower_fov', '-25.0')
        lidar_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.2))
        lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.ego_vehicle)
        lidar.listen(self.on_lidar_data)
        self.sensors.append(lidar)

        # 3. GNSS Sensor
        gnss_bp = blueprint_library.find('sensor.other.gnss')
        gnss_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.0))
        gnss = self.world.spawn_actor(gnss_bp, gnss_transform, attach_to=self.ego_vehicle)
        gnss.listen(self.on_gnss_data)
        self.sensors.append(gnss)

        # 4. IMU Sensor
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.0))
        imu = self.world.spawn_actor(imu_bp, imu_transform, attach_to=self.ego_vehicle)
        imu.listen(self.on_imu_data)
        self.sensors.append(imu)

        self.get_logger().info("Spawned 4 sensors: RGB Camera, 32-beam LiDAR, GNSS, IMU.")

    def on_camera_image(self, carla_img):
        """Processes raw CARLA camera frame and publishes sensor_msgs/Image."""
        if self.shutting_down or not rclpy.ok():
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.role_name}/camera_link'
        msg.height = carla_img.height
        msg.width = carla_img.width
        msg.encoding = 'bgra8'
        msg.is_bigendian = 0
        msg.step = carla_img.width * 4
        msg.data = bytes(carla_img.raw_data)
        try:
            self.image_pub.publish(msg)
        except Exception:
            pass

    def on_lidar_data(self, carla_lidar):
        """Converts raw CARLA LiDAR points into a ROS 2 sensor_msgs/PointCloud2."""
        if self.shutting_down or not rclpy.ok():
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = f'{self.role_name}/lidar_link'

        # CARLA lidar points are [x, y, z, intensity] in float32
        data = np.frombuffer(carla_lidar.raw_data, dtype=np.float32).reshape(-1, 4)
        if data.size == 0:
            return

        # Coordinate transform from CARLA to ROS: (x, -y, z, intensity)
        pts = np.copy(data)
        pts[:, 1] = -pts[:, 1]

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(pts)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16  # 4 * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = pts.tobytes()

        try:
            self.lidar_pub.publish(msg)
        except Exception:
            pass

    def on_gnss_data(self, carla_gnss):
        """Publishes GNSS fix."""
        if self.shutting_down or not rclpy.ok():
            return
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.role_name}/gnss_link'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = carla_gnss.latitude
        msg.longitude = carla_gnss.longitude
        msg.altitude = carla_gnss.altitude
        try:
            self.gnss_pub.publish(msg)
        except Exception:
            pass

    def on_imu_data(self, carla_imu):
        """Publishes IMU readings (angular velocity and linear acceleration)."""
        if self.shutting_down or not rclpy.ok():
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.role_name}/imu_link'

        # Angular velocity: CARLA rad/s, convert (x, -y, -z)
        msg.angular_velocity.x = carla_imu.gyroscope.x
        msg.angular_velocity.y = -carla_imu.gyroscope.y
        msg.angular_velocity.z = -carla_imu.gyroscope.z

        # Linear acceleration: CARLA m/s^2, convert (x, -y, z)
        msg.linear_acceleration.x = carla_imu.accelerometer.x
        msg.linear_acceleration.y = -carla_imu.accelerometer.y
        msg.linear_acceleration.z = carla_imu.accelerometer.z

        try:
            self.imu_pub.publish(msg)
        except Exception:
            pass

    def ackermann_callback(self, msg: AckermannDrive):
        """Receives Ackermann target commands (speed & steering angle)."""
        self.target_speed = max(0.0, msg.speed)
        self.target_steer = msg.steering_angle

    def update_telemetry_and_control(self):
        """50 Hz loop for odometry, TF publication, and vehicle control actuation."""
        if not self.ego_vehicle or not self.ego_vehicle.is_alive:
            return

        now = self.get_clock().now().to_msg()

        # Transform and velocity
        transform = self.ego_vehicle.get_transform()
        # Ignore uninitialized CARLA physics frame
        if abs(transform.location.x) < 0.1 and abs(transform.location.y) < 0.1:
            return

        vel = self.ego_vehicle.get_velocity()
        ang_vel = self.ego_vehicle.get_angular_velocity()

        # Coordinate conversion: CARLA (x, y, z) -> ROS (x, -y, z)
        x_ros = transform.location.x
        y_ros = -transform.location.y
        z_ros = transform.location.z

        roll_ros = -math.radians(transform.rotation.roll)
        pitch_ros = math.radians(transform.rotation.pitch)
        yaw_ros = -math.radians(transform.rotation.yaw)

        q = tf_transformations.quaternion_from_euler(roll_ros, pitch_ros, yaw_ros)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = f'{self.role_name}/base_link'

        odom.pose.pose.position.x = x_ros
        odom.pose.pose.position.y = y_ros
        odom.pose.pose.position.z = z_ros
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Velocity in base_link coordinates
        vx_ros = vel.x
        vy_ros = -vel.y
        vz_ros = vel.z
        self.current_speed = math.sqrt(vx_ros**2 + vy_ros**2 + vz_ros**2)

        odom.twist.twist.linear.x = self.current_speed
        odom.twist.twist.angular.z = -math.radians(ang_vel.z)

        self.odom_pub.publish(odom)

        # Broadcast TF: map -> ego_vehicle/base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = f'{self.role_name}/base_link'
        t.transform.translation.x = x_ros
        t.transform.translation.y = y_ros
        t.transform.translation.z = z_ros
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Update CARLA Spectator chase camera for live visual monitoring
        try:
            spectator = self.world.get_spectator()
            fwd = transform.get_forward_vector()
            spectator_tf = carla.Transform(
                transform.location - fwd * 7.5 + carla.Location(z=3.8),
                carla.Rotation(pitch=-16.0, yaw=transform.rotation.yaw, roll=0.0)
            )
            spectator.set_transform(spectator_tf)
        except Exception:
            pass

        # Compute vehicle actuation control
        self.apply_vehicle_control()

    def publish_static_sensor_tfs(self):
        """Publishes static child transforms for camera and lidar frames."""
        now = self.get_clock().now().to_msg()
        # Camera TF
        t_cam = TransformStamped()
        t_cam.header.stamp = now
        t_cam.header.frame_id = f'{self.role_name}/base_link'
        t_cam.child_frame_id = f'{self.role_name}/camera_link'
        t_cam.transform.translation.x = 1.8
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = 1.6
        t_cam.transform.rotation.w = 1.0

        # LiDAR TF
        t_lid = TransformStamped()
        t_lid.header.stamp = now
        t_lid.header.frame_id = f'{self.role_name}/base_link'
        t_lid.child_frame_id = f'{self.role_name}/lidar_link'
        t_lid.transform.translation.x = 0.0
        t_lid.transform.translation.y = 0.0
        t_lid.transform.translation.z = 2.2
        t_lid.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([t_cam, t_lid])

    def apply_vehicle_control(self):
        """Translates target speed and steer angle into CARLA VehicleControl."""
        control = carla.VehicleControl()

        # 1. Steering: normalize rad to [-1.0, 1.0]
        # In ROS, +steer is LEFT. In CARLA, +steer is RIGHT. Invert sign!
        normalized_steer = self.target_steer / self.max_steer_angle
        control.steer = - float(np.clip(normalized_steer, -1.0, 1.0))

        # 2. Longitudinal Speed Controller (PID)
        speed_error = self.target_speed - self.current_speed
        self.speed_integral = float(np.clip(self.speed_integral + speed_error * 0.02, -5.0, 5.0))
        speed_derivative = (speed_error - self.speed_prev_error) / 0.02
        self.speed_prev_error = speed_error

        # Active stop or Emergency Brake demand
        if self.target_speed < 0.1:
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = (self.current_speed < 0.2)
        else:
            if speed_error >= 0.0:
                throttle_cmd = (self.kp_speed * speed_error) + (self.ki_speed * self.speed_integral) + (self.kd_speed * speed_derivative)
                control.throttle = float(np.clip(throttle_cmd, 0.0, 1.0))
                control.brake = 0.0
            else:
                control.throttle = 0.0
                brake_cmd = float(np.clip(-0.5 * speed_error, 0.0, 1.0))
                control.brake = brake_cmd

        control.reverse = False
        self.ego_vehicle.apply_control(control)

    def destroy_node(self):
        """Cleans up all spawned actors upon shutdown."""
        self.shutting_down = True
        self.get_logger().info("Cleaning up CARLA actors...")
        for s in self.sensors:
            try:
                if s.is_alive:
                    s.stop()
                    s.destroy()
            except Exception as e:
                self.get_logger().warn(f"Error destroying sensor: {e}")
        self.sensors.clear()

        if self.ego_vehicle and self.ego_vehicle.is_alive:
            try:
                self.ego_vehicle.destroy()
                self.get_logger().info("Ego vehicle destroyed successfully.")
            except Exception as e:
                self.get_logger().warn(f"Error destroying vehicle: {e}")

        super().destroy_node()


def main(args=None):
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = CarlaInterfaceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
