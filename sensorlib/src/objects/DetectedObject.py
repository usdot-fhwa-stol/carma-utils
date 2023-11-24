# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

from dataclasses import dataclass
from typing import List

import carla
import numpy as np

from util.CarlaUtils import CarlaUtils



@dataclass(frozen=True)
class DetectedObject:
    """Wrapper class for carla.Actor which are detected by the sensor."""
    objectId: int
    type: str
    position: np.ndarray
    velocity: np.ndarray
    rotation: np.ndarray
    angularVelocity: np.ndarray
    positionCovariance: np.ndarray
    velocityCovariance: np.ndarray
    orientationCovariance: np.ndarray
    angularVelocityCovariance: np.ndarray  
    confidence: float
    projString: str
    size: np.ndarray
    timestamp: int
    sensorId: str
    carla_actor: carla.Actor  
    bounding_box_in_world_coordinate_frame: List[np.ndarray]


class DetectedObjectBuilder:
    @staticmethod
    def build_detected_object(carla_actor, allowed_semantic_tags, projection_string_config, sensor_Id):
        object_type = CarlaUtils.determine_object_type(carla_actor, allowed_semantic_tags)

        if (object_type == "NONE"):
            return None

        bounding_box = CarlaUtils.get_actor_bounding_box_points(carla_actor)

        if (bounding_box == None):
            return None

        
        projection_string = projection_string_config

        #TODO: replace with correct size calculation
        #https://github.com/usdot-fhwa-stol/carma-utils/issues/188
        size_x = carla_actor.bounding_box.extent.x
        size_y = carla_actor.bounding_box.extent.y
        size_z = carla_actor.bounding_box.extent.z


        return DetectedObject(
            carla_actor.id,
            object_type,
            CarlaUtils.vector3d_to_numpy(carla_actor.get_location()),
            CarlaUtils.vector3d_to_numpy(carla_actor.get_velocity()),
            CarlaUtils.get_actor_roll_pitch_yaw(carla_actor),
            CarlaUtils.get_actor_angular_velocity(carla_actor),

            # Use stand-in values which assume complete certainty
            np.zeros((3, 3)),
            np.zeros((3, 3)),
            np.zeros((3, 3)),
            np.zeros((3, 3)),

            1.0,
            projection_string,
            [size_x, size_y, size_z],
            #TODO: replace with carla sensor timestamp
            #https://github.com/usdot-fhwa-stol/carma-utils/issues/189
            0,
            sensor_Id,
            carla_actor,
            bounding_box
        )
