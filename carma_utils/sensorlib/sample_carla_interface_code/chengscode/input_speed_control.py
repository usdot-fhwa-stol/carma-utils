import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    exit()
import carla

import time
import numpy as np
import json


def spawn_vehicle(world, blueprint_name, x, y, z, yaw):
    blueprint = world.get_blueprint_library().find(blueprint_name)
    spawn_point = carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(yaw=yaw)
    )
    vehicle = world.try_spawn_actor(blueprint, spawn_point)
    if vehicle is None:
        print("Error: Vehicle could not be spawned!")
        exit()
    return vehicle

def spawn_sensor(world, blueprint_name, x, y, z, yaw, attached_obj):
    blueprint = world.get_blueprint_library().find(blueprint_name)
    lidar_transform = carla.Transform(
        carla.Location(x=x, y=y, z=z),
        carla.Rotation(yaw=yaw)
    )
    # blueprint.set_attribute('range', '100')
    sensor = world.spawn_actor(blueprint, lidar_transform, attach_to=attached_obj)
    return sensor

def process_lidar_data(world, data):
    m_data = np.frombuffer(data.raw_data, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('CosAngle', np.float32), ('ObjIdx', np.uint32),
            ('ObjTag', np.uint32)]))
    obj_idx = np.array(m_data['ObjIdx'])
    obj_tag = np.array(m_data['ObjTag'])
    vehicle_idx = obj_idx[obj_tag == 10]
    if len(vehicle_idx) > 0:
        m_dict = {}
        for idx in vehicle_idx:
            if idx not in m_dict:
                m_dict[idx] = 1
            else:
                m_dict[idx] = m_dict[idx] + 1
        for key in m_dict:
            print(world.get_actor(int(key)))
    
    # for detection in data:
    #     print(detection)
        # actor = world.get_actor(detection.object_idx)
        # if actor is not None:
        #     print(actor)
        

def move_vehicle_with_point(world, vehicle, point, velocity):
    vehicle.enable_constant_velocity(carla.Vector3D(x=velocity, y=0.0, z=0.0))
    # print(type(vehicle.get_location()))
    index = next(i for i, x in enumerate(point) if x != 0)
    while True:
        world.tick()
        location = vehicle.get_location()
        velocity = vehicle.get_velocity()
        current_speed = 2.23694 * (velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) ** 0.5
        tmp = [0,0,0]
        tmp_loc = [location.x, location.y, location.z]
        for i in range(len(tmp_loc)):
            if i == index:
                tmp[i] = point[i]
                continue
            else:
                tmp[i] = tmp_loc[i]

        if ((tmp[0]-tmp_loc[0])**2 + (tmp[1]-tmp_loc[1])**2 + (tmp[2]-tmp_loc[2])**2) ** 0.5 <= 1:
            break
    vehicle.enable_constant_velocity(carla.Vector3D(x=0.0, y=0.0, z=0.0))
    time.sleep(1)
    vehicle.destroy()

def read_json(json_path):
    file = open(json_path)
    return json.load(file)

if __name__ == '__main__':

    # Read json config
    config = read_json("data.json")

    # Connect to the simulator and retrieve the world
    client = carla.Client(config["CARLAIp"], config["CARLAPort"])
    client.set_timeout(2.0)
    world = client.get_world()
    for vehicle_cfg in config["vehicles"]:
        x_pt = vehicle_cfg["startPt"][0]
        y_pt = vehicle_cfg["startPt"][1]
        z_pt = vehicle_cfg["startPt"][2]
        yaw = vehicle_cfg["yaw"]
        # Spawn the vehicle at the specified location with the initial speed
        vehicle = spawn_vehicle(world, vehicle_cfg["vehicleModel"], x_pt, y_pt, z_pt, yaw)

        # Spawn the lidar at the specified vehicle
        lidar = spawn_sensor(world, 'sensor.lidar.ray_cast_semantic', 1, 0, 3, 180, vehicle)
        lidar.listen(lambda data: process_lidar_data(world, data))

        # Set CARLA simulator as wall clock
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = None # Set a variable time-step
        world.apply_settings(settings)

        move_vehicle_with_point(world, vehicle, vehicle_cfg['endPt'], vehicle_cfg['targetSpeedMPH'] / 2.23694)

        settings.synchronous_mode = False
        world.apply_settings(settings)
