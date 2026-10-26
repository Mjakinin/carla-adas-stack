#!/usr/bin/env python3
import carla
import time
import math
import subprocess
import signal
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

    # Get spawn point
    sp0 = world.get_map().get_spawn_points()[0]
    wp = world.get_map().get_waypoint(sp0.location)

    # Spawn an obstacle vehicle 35 meters ahead in the ego vehicle's lane
    obs_wp = wp.next(35.0)[0]
    obs_tf = carla.Transform(
        carla.Location(obs_wp.transform.location.x, obs_wp.transform.location.y, obs_wp.transform.location.z + 0.5),
        obs_wp.transform.rotation
    )
    obs_bp = world.get_blueprint_library().find('vehicle.audi.tt')
    obs_vehicle = world.spawn_actor(obs_bp, obs_tf)
    print(f"Spawned stationary obstacle Audi TT at: ({obs_tf.location.x:.1f}, {obs_tf.location.y:.1f})")

    # Launch ADAS stack
    print("Launching ADAS stack...")
    cmd = "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && ros2 launch carla_adas_stack carla_adas.launch.py"
    proc = subprocess.Popen(cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    try:
        # Wait for ego vehicle
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

        print(f"Ego vehicle spawned: id={ego.id}")

        stopped_cleanly = False
        final_distance = None

        # Telemetry monitor loop for 12 seconds
        for i in range(24):
            time.sleep(0.5)
            ego_loc = ego.get_location()
            obs_loc = obs_vehicle.get_location()
            dist = math.hypot(ego_loc.x - obs_loc.x, ego_loc.y - obs_loc.y)
            vel = ego.get_velocity()
            spd = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            ctrl = ego.get_control()

            print(f"[{i*0.5:4.1f}s] Speed: {spd:4.1f} km/h | Obstacle Dist: {dist:5.2f}m | Throttle: {ctrl.throttle:.2f} | Brake: {ctrl.brake:.2f}")

            if i > 6 and spd < 0.2:
                stopped_cleanly = True
                final_distance = dist
                print(f"--> [AEB SUCCESS] Ego vehicle brought to complete stop at distance {dist:.2f}m from obstacle!")
                break

        if not stopped_cleanly:
            print(f"--> [TEST RESULT] Finished monitoring, ego stopped: {stopped_cleanly}")

    finally:
        print("Cleaning up...")
        obs_vehicle.destroy()
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        print("Done.")

if __name__ == '__main__':
    main()
