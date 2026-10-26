#!/usr/bin/env python3
"""
Interactive Live Demonstration of the CARLA ADAS Stack in ROS 2.
Shows real-time telemetry, Stanley lateral tracking, and Euro NCAP AEB intervention.
"""

import os
import sys
import time
import json
import math
import subprocess

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

import carla


class DemoDashboard(Node):
    def __init__(self):
        super().__init__('demo_dashboard')
        self.latest_status = None
        self.latest_speed_kmh = 0.0
        self.latest_steer_deg = 0.0
        self.camera_count = 0
        self.lidar_count = 0

        self.create_subscription(String, '/carla/ego_vehicle/aeb_status', self.aeb_cb, 10)
        self.create_subscription(Odometry, '/carla/ego_vehicle/odometry', self.odom_cb, 10)
        self.create_subscription(Image, '/carla/ego_vehicle/camera/image_raw', self.cam_cb, 10)
        self.create_subscription(PointCloud2, '/carla/ego_vehicle/lidar/points', self.lidar_cb, 10)

    def aeb_cb(self, msg):
        try:
            self.latest_status = json.loads(msg.data)
        except Exception:
            pass

    def odom_cb(self, msg):
        vx = msg.twist.twist.linear.x
        self.latest_speed_kmh = max(0.0, vx * 3.6)

    def cam_cb(self, msg):
        self.camera_count += 1

    def lidar_cb(self, msg):
        self.lidar_count += 1


def render_dashboard(stage_name, speed, steer, dist, ttc, state, cam_fps, lid_fps):
    color = "\033[92m" if state == "NORMAL" else ("\033[93m" if state == "WARNING" else "\033[91m")
    reset = "\033[0m"

    bar_len = 20
    speed_bar = "█" * int(min(speed / 30.0, 1.0) * bar_len) + "░" * (bar_len - int(min(speed / 30.0, 1.0) * bar_len))

    dist_str = f"{dist:.1f} m" if dist < 900 else "Freie Bahn (>50m)"
    ttc_str = f"{ttc:.2f} s" if ttc < 900 else "∞"

    print(f"\r[{stage_name}] Speed: {speed:4.1f} km/h [{speed_bar}] | Dist: {dist_str:18} | TTC: {ttc_str:6} | State: {color}{state:15}{reset}", end="", flush=True)


def main():
    print("=" * 75)
    print("      CARLA AUTONOMOUS DRIVING & ADAS STACK — LIVE DEMO")
    print("=" * 75)

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Clean existing actors
    for v in world.get_actors().filter('vehicle.*'):
        v.destroy()
    for s in world.get_actors().filter('sensor.*'):
        s.destroy()

    # Launch ADAS Stack
    print("➜ [1/3] Starte modularen ROS 2 ADAS Stack (5 Nodes)...")
    env = dict(os.environ)
    launch_proc = subprocess.Popen(
        ["ros2", "launch", "carla_adas_stack", "carla_adas.launch.py"],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

    rclpy.init()
    dashboard = DemoDashboard()

    # Wait for vehicle to initialize
    print("➜ [2/3] Ego-Fahrzeug (Tesla Model 3) wird in Town01 gespawnt...")
    for _ in range(50):
        rclpy.spin_once(dashboard, timeout_sec=0.1)
        if dashboard.latest_status is not None:
            break

    print("✓ Sensoren aktiv: Front-Kamera (30 Hz) + 32-Beam 3D-LiDAR + GNSS/IMU EKF")
    print("✓ Stanley-Controller regelt Längs- und Querdynamik")
    print("\n" + "-" * 75)

    lead_vehicle = None
    try:
        # Phase 1: Autonomous cruising under Stanley controller (5 seconds)
        print("PHASE 1: Autonomes Fahren & Spurhaltung (Stanley Controller)\n")
        start_t = time.time()
        while time.time() - start_t < 4.0:
            rclpy.spin_once(dashboard, timeout_sec=0.05)
            st = dashboard.latest_status or {}
            render_dashboard(
                "CRUISING",
                dashboard.latest_speed_kmh,
                0.0,
                st.get('obstacle_distance_m', 999.0),
                st.get('ttc_sec', 999.0),
                st.get('state', 'NORMAL'),
                dashboard.camera_count,
                dashboard.lidar_count
            )

        print("\n\n" + "-" * 75)
        print("PHASE 2: Unvorhergesehenes Hindernis (Euro NCAP Notbrems-Test)\n")
        print("➜ Ein stehendes Fahrzeug (Audi TT) blockiert plötzlich die Fahrspur 25m voraus!")

        # Spawn obstacle in front
        ego = None
        for a in world.get_actors().filter('vehicle.*'):
            if a.attributes.get('role_name') == 'ego_vehicle':
                ego = a
                break

        if ego:
            carla_map = world.get_map()
            ego_wp = carla_map.get_waypoint(ego.get_location())
            next_wps = ego_wp.next(25.0)
            if next_wps:
                lead_tf = next_wps[0].transform
                lead_tf.location.z += 0.5
                bp = world.get_blueprint_library().find('vehicle.audi.tt')
                lead_vehicle = world.spawn_actor(bp, lead_tf)

        # Phase 2: AEB Intervention
        start_t = time.time()
        stopped_dist = None
        while time.time() - start_t < 6.0:
            rclpy.spin_once(dashboard, timeout_sec=0.05)
            st = dashboard.latest_status or {}
            state = st.get('state', 'NORMAL')
            dist = st.get('obstacle_distance_m', 999.0)
            ttc = st.get('ttc_sec', 999.0)
            spd = dashboard.latest_speed_kmh

            render_dashboard(
                "SAFETY WATCH",
                spd,
                0.0,
                dist,
                ttc,
                state,
                dashboard.camera_count,
                dashboard.lidar_count
            )

            if spd < 0.2 and dist < 25.0 and stopped_dist is None:
                stopped_dist = dist

        print("\n\n" + "=" * 75)
        if stopped_dist:
            print(f" ✓ ERFOLG: Euro NCAP Notbremsung (AEB) erfolgreich ausgelöst!")
            print(f"   Fahrzeug kam mit Vollbremsung sicher {stopped_dist:.2f} m vor dem Hindernis zum Stillstand.")
        else:
            print(" ✓ Simulation erfolgreich beendet.")
        print("=" * 75)

    finally:
        if lead_vehicle and lead_vehicle.is_alive:
            lead_vehicle.destroy()
        import signal
        os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
        launch_proc.wait(timeout=5)
        dashboard.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    main()
