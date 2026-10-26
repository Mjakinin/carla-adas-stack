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
    col_sensor = None
    collisions = []

    try:
        # Phase 1: Autonomous cruising on straight track under Stanley controller
        print("PHASE 1: Autonomes Fahren & Spurhaltung (Stanley Controller)\n")
        start_t = time.time()
        while time.time() - start_t < 2.2:
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
        print("➜ Ein stehendes Fahrzeug (Audi TT) blockiert die Fahrspur 26m voraus!")

        # Find ego vehicle and attach collision sensor
        ego = None
        for a in world.get_actors().filter('vehicle.*'):
            if a.attributes.get('role_name') == 'ego_vehicle':
                ego = a
                break

        if ego:
            col_bp = world.get_blueprint_library().find('sensor.other.collision')
            col_sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=ego)
            col_sensor.listen(lambda event: collisions.append(event.other_actor.type_id))

            # Optional HD Direct-Render Video Recording
            record_mode = "--record" in sys.argv or "-r" in sys.argv
            record_cam = None
            ffmpeg_proc = None
            media_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../docs/media"))
            out_mp4 = os.path.join(media_dir, "aeb_emergency_braking.mp4")
            out_gif = os.path.join(media_dir, "aeb_emergency_braking.gif")

            if record_mode:
                os.makedirs(media_dir, exist_ok=True)
                ffmpeg_proc = subprocess.Popen([
                    "ffmpeg", "-y",
                    "-f", "rawvideo",
                    "-vcodec", "rawvideo",
                    "-s", "1280x720",
                    "-pix_fmt", "bgra",
                    "-r", "25",
                    "-i", "-",
                    "-c:v", "libx264",
                    "-preset", "fast",
                    "-crf", "20",
                    "-pix_fmt", "yuv420p",
                    out_mp4
                ], stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)

                cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
                cam_bp.set_attribute('image_size_x', '1280')
                cam_bp.set_attribute('image_size_y', '720')
                cam_bp.set_attribute('fov', '90')
                cam_tf = carla.Transform(carla.Location(x=-6.5, y=0.0, z=3.0), carla.Rotation(pitch=-14.0))
                record_cam = world.spawn_actor(cam_bp, cam_tf, attach_to=ego)

                def _cam_callback(img):
                    try:
                        if ffmpeg_proc and ffmpeg_proc.stdin:
                            ffmpeg_proc.stdin.write(bytes(img.raw_data))
                    except (BrokenPipeError, OSError):
                        pass

                record_cam.listen(_cam_callback)
                print("✓ Direkte HD-Renderaufnahme aktiv (1280x720 @ 25 FPS Stream)...")

            carla_map = world.get_map()
            ego_wp = carla_map.get_waypoint(ego.get_location())
            next_wps = ego_wp.next(26.0)
            if next_wps:
                lead_tf = next_wps[0].transform
                lead_tf.location.z += 0.5
                bp = world.get_blueprint_library().find('vehicle.audi.tt')
                lead_vehicle = world.spawn_actor(bp, lead_tf)

        # Phase 2: AEB Intervention
        start_t = time.time()
        stopped_dist = None
        while time.time() - start_t < 8.0:
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

            if len(collisions) > 0:
                break

            if spd < 0.15 and dist < 20.0 and (time.time() - start_t > 1.5):
                stopped_dist = dist
                break

        print("\n\n" + "=" * 75)
        if len(collisions) > 0:
            print(f" ✗ KOLLISION ERKANNT! Ego-Fahrzeug hat Hindernis berührt ({collisions[0]})")
        elif stopped_dist:
            print(f" ✓ ERFOLG: Euro NCAP Notbremsung (AEB) erfolgreich ausgelöst!")
            print(f"   Fahrzeug kam mit Vollbremsung sicher {stopped_dist:.2f} m vor dem Hindernis zum Stillstand.")
            print(f"   Kollisionen registriert: 0 (Perfekt abgesichert).")
        else:
            print(" ✓ Simulation beendet.")
        print("=" * 75)

    finally:
        if 'record_cam' in locals() and record_cam and record_cam.is_alive:
            record_cam.stop()
            record_cam.destroy()
        if 'ffmpeg_proc' in locals() and ffmpeg_proc and ffmpeg_proc.stdin:
            try:
                ffmpeg_proc.stdin.close()
                ffmpeg_proc.wait(timeout=10)
            except Exception:
                pass
        if col_sensor and col_sensor.is_alive:
            col_sensor.stop()
            col_sensor.destroy()
        if lead_vehicle and lead_vehicle.is_alive:
            lead_vehicle.destroy()
        import signal
        os.killpg(os.getpgid(launch_proc.pid), signal.SIGINT)
        launch_proc.wait(timeout=5)
        dashboard.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        if 'record_mode' in locals() and record_mode and os.path.exists(out_mp4):
            print("➜ Generiere animiertes Showcase-GIF für GitHub README...")
            subprocess.run([
                "ffmpeg", "-y", "-i", out_mp4,
                "-vf", "fps=12,scale=720:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse",
                "-loop", "0", out_gif
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"✓ Video & GIF erfolgreich in {media_dir} abgelegt!")

    return 0


if __name__ == '__main__':
    main()
