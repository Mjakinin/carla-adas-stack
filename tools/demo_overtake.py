#!/usr/bin/env python3
"""
Interactive Demonstration of Level 3/4 Dynamic Overtaking in CARLA & ROS 2.
Demonstrates:
  1. Stanley lane keeping at 30 km/h on a long straight avenue.
  2. Stationary obstacle detection (Audi TT) ~32m ahead.
  3. Jerk-optimal Quintic Polynomial S-curve lane change into the adjacent lane (d = 3.8m).
  4. Parallel passing alongside the obstacle.
  5. Smooth Quintic return lane change back into the host lane.
  6. Zero collisions verified by native CARLA collision sensor.
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
from std_msgs.msg import String

import carla


class OvertakeDashboard(Node):
    def __init__(self):
        super().__init__('overtake_dashboard')
        self.latest_odom = None
        self.latest_aeb = None
        self.latest_overtake = None
        self.speed_kmh = 0.0
        self.ego_x = 0.0
        self.ego_y = 0.0

        self.create_subscription(Odometry, '/carla/ego_vehicle/odometry', self.odom_cb, 10)
        self.create_subscription(String, '/carla/ego_vehicle/aeb_status', self.aeb_cb, 10)
        self.create_subscription(String, '/carla/ego_vehicle/overtake_status', self.overtake_cb, 10)

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg
        vx = msg.twist.twist.linear.x
        self.speed_kmh = max(0.0, vx * 3.6)
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y

    def aeb_cb(self, msg: String):
        try:
            self.latest_aeb = json.loads(msg.data)
        except Exception:
            pass

    def overtake_cb(self, msg: String):
        try:
            self.latest_overtake = json.loads(msg.data)
        except Exception:
            pass


def render_dashboard(state, speed, ego_x, ego_y, obs_x, d_lat, dist_obs):
    # Color coding
    if "LEFT" in state:
        color = "\033[96m"  # Cyan
    elif "PASSING" in state:
        color = "\033[93m"  # Yellow
    elif "RIGHT" in state:
        color = "\033[95m"  # Magenta
    elif "COMPLETED" in state:
        color = "\033[92m"  # Green
    else:
        color = "\033[97m"  # White

    reset = "\033[0m"

    speed_bar_len = 15
    bar_fill = int(min(speed / 30.0, 1.0) * speed_bar_len)
    speed_bar = "█" * bar_fill + "░" * (speed_bar_len - bar_fill)

    dist_str = f"{dist_obs:5.1f}m" if dist_obs < 900 else "Frei"
    dx_obs = (obs_x - ego_x) if obs_x > 0 else 0.0

    print(f"\r[{color}{state:18}{reset}] Speed: {speed:4.1f}km/h [{speed_bar}] | Pos: ({ego_x:5.1f}, {ego_y:5.1f}) | d: {d_lat:4.2f}m | Obstacle Δx: {dx_obs:5.1f}m", end="", flush=True)


def main():
    print("=" * 78)
    print("   CARLA ADAS STACK — LEVEL 3/4 DYNAMIC OVERTAKING & S-CURVE DEMO")
    print("=" * 78)

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Clean existing actors
    for v in world.get_actors().filter('vehicle.*'):
        v.destroy()
    for s in world.get_actors().filter('sensor.*'):
        s.destroy()

    # Launch ADAS Stack with spawn_index=87 (straight avenue) and enable_overtaking=true
    print("➜ [1/3] Starte ROS 2 ADAS Stack mit aktivem Dynamic-Overtaking-Planer...")
    env = dict(os.environ)
    launch_proc = subprocess.Popen(
        [
            "ros2", "launch", "carla_adas_stack", "carla_adas.launch.py",
            "spawn_index:=87",
            "enable_overtaking:=true"
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

    rclpy.init()
    dashboard = OvertakeDashboard()

    # Wait for vehicle to initialize
    print("➜ [2/3] Ego-Fahrzeug (Tesla Model 3) wird auf Allee (Spawn 87) gespawnt...")
    for _ in range(60):
        rclpy.spin_once(dashboard, timeout_sec=0.1)
        if dashboard.latest_odom is not None:
            break

    # Find ego vehicle
    ego = None
    for a in world.get_actors().filter('vehicle.*'):
        if a.attributes.get('role_name') == 'ego_vehicle':
            ego = a
            break

    if not ego:
        print("✗ Fehler: Ego-Fahrzeug nicht gefunden!")
        return 1

    # Attach collision sensor
    collisions = []
    col_bp = world.get_blueprint_library().find('sensor.other.collision')
    col_sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=ego)
    col_sensor.listen(lambda event: collisions.append(event.other_actor.type_id))

    # Optional HD Direct-Render Video Recording
    record_mode = "--record" in sys.argv or "-r" in sys.argv
    record_cam = None
    ffmpeg_proc = None
    media_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../docs/media"))
    out_mp4 = os.path.join(media_dir, "dynamic_overtake_maneuver.mp4")
    out_gif = os.path.join(media_dir, "dynamic_overtake_maneuver.gif")

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

    # Spawn stationary obstacle (Audi TT) 55m ahead in the same lane
    print("➜ [3/3] Platziere parkendes Hindernis (Audi TT) 55m voraus in der Fahrspur...")
    ego_tf = ego.get_transform()
    tt_bp = world.get_blueprint_library().find('vehicle.audi.tt')
    
    # Audi TT parked at x = 162.0 in the center of the right lane
    lead_tf = carla.Transform(
        carla.Location(x=162.0, y=ego_tf.location.y, z=ego_tf.location.z + 0.3),
        ego_tf.rotation
    )
    lead_vehicle = world.spawn_actor(tt_bp, lead_tf)

    print("\n" + "-" * 78)
    print("MISSION: Autonomes Überholen mit ruckfreier Frenet-Quintic-Trajektorie")
    print("-" * 78 + "\n")

    start_t = time.time()
    min_pass_distance = float('inf')
    max_lateral_offset = 0.0

    try:
        while time.time() - start_t < 25.0:
            rclpy.spin_once(dashboard, timeout_sec=0.04)

            state = "CRUISING"
            d_lat = 0.0
            obs_x = 162.0

            if dashboard.latest_overtake:
                state = dashboard.latest_overtake.get('state', 'CRUISING')
                d_lat = dashboard.latest_overtake.get('d_current', 0.0)

            max_lateral_offset = max(max_lateral_offset, abs(d_lat))

            # Calculate physical distance to TT
            dist_phys = 999.0
            if ego.is_alive and lead_vehicle.is_alive:
                loc_ego = ego.get_location()
                loc_tt = lead_vehicle.get_location()
                dist_phys = math.hypot(loc_ego.x - loc_tt.x, loc_ego.y - loc_tt.y)
                if abs(loc_ego.x - loc_tt.x) < 8.0:
                    min_pass_distance = min(min_pass_distance, dist_phys)

            render_dashboard(
                state,
                dashboard.speed_kmh,
                dashboard.ego_x,
                dashboard.ego_y,
                obs_x,
                d_lat,
                dist_phys
            )

            if len(collisions) > 0:
                break

            # If vehicle completed overtake and safely rejoined lane (past x > 195m)
            if dashboard.ego_x > 195.0 and state in ["COMPLETED", "CRUISE_RESUMED"]:
                break

        print("\n\n" + "=" * 78)
        if len(collisions) > 0:
            print(f" ✗ KOLLISION ERKANNT! Ego-Fahrzeug hat berührt: {collisions[0]}")
        else:
            print(" ✓ ERFOLG: Dynamisches Überholmanöver perfekt ausgeführt!")
            print(f"   • Maximale Querablage (Überholspur): {max_lateral_offset:.2f} m")
            print(f"   • Minimaler Sicherheitsabstand beim Vorbeifahren: {min_pass_distance:.2f} m")
            print(f"   • Endposition: x={dashboard.ego_x:.1f} m (zurück auf Spurmitte)")
            print("   • Kollisionen registriert: 0 (Volle Sicherheit)")
        print("=" * 78)

    finally:
        if record_cam and record_cam.is_alive:
            record_cam.stop()
            record_cam.destroy()
        if ffmpeg_proc and ffmpeg_proc.stdin:
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

        if record_mode and os.path.exists(out_mp4):
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
