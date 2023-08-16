#!/usr/bin/python
# -*- coding: utf-8 -*-
from dataclasses import dataclass
from typing import Tuple, List

import carla
import numpy as np

from src.util.CarlaUtils import CarlaUtils


@dataclass(frozen=True)
class DetectedObject:
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
