#!/usr/bin/env python3
"""
Test script for Euro NCAP AEB (Autonomous Emergency Braking) scenario.
Spawns a lead vehicle stopped in the lane ahead of the ego vehicle.
Monitors the ego vehicle telemetry, TTC, and verifies successful emergency stop.
"""

import time
import carla

def run_aeb_test():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # Find ego vehicle
    ego = None
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name') == 'ego_vehicle':
            ego = actor
            break

    if not ego:
        print("Ego vehicle not found! Is carla_adas.launch.py running?")
        return

    ego_transform = ego.get_transform()
    print(f"Ego Vehicle found at: ({ego_transform.location.x:.2f}, {ego_transform.location.y:.2f})")

    # Calculate position 25 meters directly ahead along road lane
    carla_map = world.get_map()
    ego_wp = carla_map.get_waypoint(ego_transform.location)
    next_wps = ego_wp.next(25.0)
    if not next_wps:
        print("No lane waypoint found ahead.")
        return

    lead_transform = next_wps[0].transform
    lead_transform.location.z += 0.5

    bp = world.get_blueprint_library().find('vehicle.audi.tt')
    lead_vehicle = world.try_spawn_actor(bp, lead_transform)
    if not lead_vehicle:
        lead_transform.location.z += 0.5
        lead_vehicle = world.spawn_actor(bp, lead_transform)
    print(f"Spawned stationary lead vehicle '{bp.id}' 25m ahead at ({lead_transform.location.x:.2f}, {lead_transform.location.y:.2f})")

    try:
        for i in range(20):
            ego_loc = ego.get_location()
            lead_loc = lead_vehicle.get_location()
            dist = ego_loc.distance(lead_loc)
            vel = ego.get_velocity()
            speed_kmh = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            print(f"[{i*0.5:.1f}s] Distance to Obstacle: {dist:.2f} m | Ego Speed: {speed_kmh:.1f} km/h")
            if speed_kmh < 0.5 and dist < 12.0:
                print(f"✓ AEB EMERGENCY BRAKING SUCCESSFUL! Vehicle stopped safely {dist:.2f} m before collision!")
                break
            time.sleep(0.5)
    finally:
        lead_vehicle.destroy()
        print("Obstacle vehicle cleaned up.")

if __name__ == '__main__':
    import math
    run_aeb_test()
