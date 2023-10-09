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

from src.util.CarlaUtils import CarlaUtils


@dataclass(frozen=True)
class DetectedObject:
    """Wrapper class for carla.Actor which are detected by the sensor."""
    carla_actor: carla.Actor
    id: int
    object_type: str
    timestamp: int
    bounding_box_in_world_coordinate_frame: List[np.ndarray]
    position: np.ndarray
    velocity: np.ndarray
    rotation: np.ndarray
    angular_velocity: np.ndarray
    position_covariance: np.ndarray
    velocity_covariance: np.ndarray
    confidence: float


class DetectedObjectBuilder:
    @staticmethod
    def build_detected_object(carla_actor, allowed_semantic_tags):
        return DetectedObject(
            carla_actor,
            carla_actor.id,
            CarlaUtils.determine_object_type(carla_actor, allowed_semantic_tags),
            0,
            CarlaUtils.get_actor_bounding_box_points(carla_actor),
            CarlaUtils.vector3d_to_numpy(carla_actor.get_location()),
            CarlaUtils.vector3d_to_numpy(carla_actor.get_velocity()),
            CarlaUtils.get_actor_rotation_matrix(carla_actor),
            CarlaUtils.get_actor_angular_velocity(carla_actor),

            # Use stand-in values which assume complete certainty
            np.zeros((3, 3)),
            np.zeros((3, 3)),
            1.0
        )
