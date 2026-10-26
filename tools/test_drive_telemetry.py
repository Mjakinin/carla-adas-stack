#!/usr/bin/env python3
import carla
import time
import math
import subprocess
import signal
import sys
import os

def main():
    print("Connecting to CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Clean world
    print("Cleaning existing actors...")
    for a in world.get_actors().filter('vehicle.*'): a.destroy()
    for s in world.get_actors().filter('sensor.*'): s.destroy()

    # Launch ROS 2 stack as a subprocess group
    print("Launching ADAS stack...")
    cmd = "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && ros2 launch carla_adas_stack carla_adas.launch.py"
    proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    try:
        # Wait for vehicle to spawn
        ego = None
        for _ in range(20):
            time.sleep(0.5)
            for a in world.get_actors().filter('vehicle.*'):
                if a.attributes.get('role_name') == 'ego_vehicle':
                    ego = a
                    break
            if ego:
                break

        if not ego:
            print("ERROR: Ego vehicle was not spawned within 10 seconds!")
            return

        print(f"Ego vehicle spawned: id={ego.id}, loc=({ego.get_location().x:.2f}, {ego.get_location().y:.2f})")
        carla_map = world.get_map()

        # Telemetry monitor loop for 10 seconds (20 iterations of 0.5s)
        for i in range(20):
            time.sleep(0.5)
            loc = ego.get_location()
            rot = ego.get_transform().rotation
            vel = ego.get_velocity()
            spd = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            ctrl = ego.get_control()

            wp = carla_map.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            wp_loc = wp.transform.location
            dist_to_center = math.hypot(loc.x - wp_loc.x, loc.y - wp_loc.y)

            print(f"[{i*0.5:4.1f}s] Speed: {spd:4.1f} km/h | Pos: (x={loc.x:5.1f}, y={loc.y:5.1f}) | Yaw: {rot.yaw:5.1f}° | Steer: {ctrl.steer:+5.2f} | Throttle: {ctrl.throttle:.2f} | LaneDist: {dist_to_center:4.2f}m")

    finally:
        print("Terminating ROS 2 stack...")
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        print("Done.")

if __name__ == '__main__':
    main()
