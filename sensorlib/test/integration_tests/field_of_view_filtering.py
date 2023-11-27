# Copyright 2023 Leidos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import defaultdict
import sys
import time

import carla

from src.CarlaCDASimAPI import CarlaCDASimAPI

NOISE_MODEL_CONFIG = {
    "noise_model_name": "GaussianNoiseModel",
    "stages": {
        "position_noise": True,
        "orientation_noise": True,
        "type_noise": False,
        "list_inclusion_noise": False,
    },
    "std_deviations": {"position_in_meters": [0.8, 0.8, 0.8], "orientation_in_radians": [0.1, 0.1, 0.1]},
    "type_noise": {
        "allowed_semantic_tags": {
            "Buildings",
            "Fences",
            "Ground",
            "GuardRail",
            "Pedestrians",
            "Poles",
            "RoadLines",
            "Roads",
            "Sidewalks",
            "Sky",
            "Terrain",
            "TrafficLight",
            "TrafficSigns",
            "Vegetation",
            "Vehicles",
            "Walls",
            "Water",
        }
    },
}

DETECTION_CYCLE_DELAY_SECONDS = 0.5


def set_blueprint_attributes(blueprint, attributes):
    blueprint.set_attribute("lower_fov", str(attributes["lower_fov"]))
    blueprint.set_attribute("upper_fov", str(attributes["upper_fov"]))
    blueprint.set_attribute("channels", str(attributes["channels"]))
    blueprint.set_attribute("range", str(attributes["range"]))

    # CARLA 0.9.10 uses 'rotation_frequency' instead of 'rotation_period'
    # dummy_lidar_blueprint.set_attribute("rotation_period", "0.1")
    blueprint.set_attribute(
        "rotation_frequency", str(1 / attributes["rotation_period"])
    )

    blueprint.set_attribute("points_per_second", str(attributes["points_per_second"]))

    return blueprint


def spawn_traffic_light_lidar(world):
    simulated_sensor_config = {
        "prefilter": {
            "allowed_semantic_tags": ["Vehicles", "Pedestrians"],
            "max_distance_meters": 20.0,
        },
        "detection_threshold_scaling_formula": {
            "nominal_hitpoint_detection_ratio_threshold": 0.6,
            "hitpoint_detection_ratio_threshold_per_meter_change_rate": -0.0033,
            "adjustable_threshold_scaling_parameters": {"dropoff_rate": 0.01},
        },
        "geometry_reassociation": {
            "sample_count": 3,
            "geometry_association_max_dist_in_meters": 2.0,
        },
        "use_sensor_centric_frame": True,
    }

    carla_sensor_config = {
        "lower_fov": -80.0,
        "upper_fov": 30.0,
        "channels": 32,
        "range": 20.0,
        "rotation_period": 0.1,
        "points_per_second": 56000,
    }

    # These values have no real meaning. They are used as keys in an
    # internal dict. Their tuple must be unique.
    infrastructure_id = 1
    sensor_id = 2

    spawn_point = carla.Transform(
        carla.Location(191.60, -253.10, 3.0), carla.Rotation(0.0, 0.0, 0.0)
    )

    blueprint_library = world.get_blueprint_library()
    dummy_lidar_blueprint = blueprint_library.find("sensor.lidar.ray_cast")
    set_blueprint_attributes(dummy_lidar_blueprint, carla_sensor_config)
    dummy_lidar = world.spawn_actor(dummy_lidar_blueprint, spawn_point)

    api = CarlaCDASimAPI.build_from_world(world)

    return (
        api.create_simulated_semantic_lidar_sensor(
            simulated_sensor_config,
            carla_sensor_config,
            NOISE_MODEL_CONFIG,
            DETECTION_CYCLE_DELAY_SECONDS,
            infrastructure_id,
            sensor_id,
            spawn_point.location,
            spawn_point.rotation,
        ),
        dummy_lidar,
    )


def spawn_vehicle_lidar(world):
    simulated_sensor_config = {
        "prefilter": {
            "allowed_semantic_tags": ["Vehicles", "Pedestrians"],
            "max_distance_meters": 10.0,
        },
        "detection_threshold_scaling_formula": {
            "nominal_hitpoint_detection_ratio_threshold": 0.6,
            "hitpoint_detection_ratio_threshold_per_meter_change_rate": -0.0033,
            "adjustable_threshold_scaling_parameters": {"dropoff_rate": 0.01},
        },
        "geometry_reassociation": {
            "sample_count": 3,
            "geometry_association_max_dist_in_meters": 2.0,
        },
        "use_sensor_centric_frame": True,
    }

    carla_sensor_config = {
        "lower_fov": -80.0,
        "upper_fov": 30.0,
        "channels": 32,
        "range": 10.0,
        "rotation_period": 0.1,
        "points_per_second": 56000,
    }

    # These values have no real meaning. They are used as keys in an
    # internal dict. Their tuple must be unique.
    infrastructure_id = 2
    sensor_id = 3

    ego_vehicle = None
    for actor in world.get_actors():
        if (
            "role_name" in actor.attributes
            and "ego_vehicle" in actor.attributes["role_name"]
        ):
            ego_vehicle = actor
            print(f"Using actor with ID: {ego_vehicle.id}")
            break
    else:
        raise RuntimeError("no actor with role name 'ego_vehicle'")

    lidar_offset = carla.Transform(carla.Location(z=1.5))

    blueprint_library = world.get_blueprint_library()
    dummy_lidar_blueprint = blueprint_library.find("sensor.lidar.ray_cast")
    set_blueprint_attributes(dummy_lidar_blueprint, carla_sensor_config)
    dummy_lidar = world.spawn_actor(
        dummy_lidar_blueprint, lidar_offset, attach_to=ego_vehicle
    )

    api = CarlaCDASimAPI.build_from_world(world)

    return (
        api.create_simulated_semantic_lidar_sensor(
            simulated_sensor_config,
            carla_sensor_config,
            NOISE_MODEL_CONFIG,
            DETECTION_CYCLE_DELAY_SECONDS,
            infrastructure_id,
            sensor_id,
            lidar_offset.location,
            lidar_offset.rotation,
            parent_id=ego_vehicle.id,
        ),
        dummy_lidar,
    )


def print_detection_summary(lidar_name, lidar):
    actor_types = defaultdict(int)

    for detected_object in lidar.get_detected_objects():
        actor_types[str(detected_object.type)] += 1

    print(f"'{lidar_name}' detection summary by actor type:")
    print("\n".join([f"{key}, {value}" for key, value in actor_types.items()]))
    print("")


def main() -> int:
    carla_client = carla.Client("localhost", 2000)
    world = carla_client.get_world()

    traffic_light_lidar, tf_dummy = spawn_traffic_light_lidar(world)
    vehicle_lidar, v_dummy = spawn_vehicle_lidar(world)

    iterations = 0
    while iterations < 20:
        print_detection_summary("traffic_light_lidar", traffic_light_lidar)
        print_detection_summary("vehicle_lidar", vehicle_lidar)

        iterations += 1
        time.sleep(1.0)

    # Note this script will not destroy any spawned actors.

    return 0


if __name__ == "__main__":
    sys.exit(main())
