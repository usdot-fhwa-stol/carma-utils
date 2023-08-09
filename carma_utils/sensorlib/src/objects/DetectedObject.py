#!/usr/bin/python
# -*- coding: utf-8 -*-
from dataclasses import dataclass
from typing import Tuple

import carla
import numpy as np

from src.util.CarlaUtils import CarlaUtils


@dataclass(frozen=True)
class DetectedObject:
    carla_actor: carla.Actor
    id: int
    object_type: str
    size: Tuple[float, float, float]
    position: np.ndarray  # Length, width, height of object in meters
    velocity: np.ndarray
    rotation: np.ndarray
    angular_velocity: np.ndarray
    position_covariance: np.ndarray
    velocity_covariance: np.ndarray
    confidence: float

class DetectedObjectBuilder:
    @staticmethod
    def build_detected_object(carla_actor, object_type):
        return DetectedObject(
            carla_actor,
            carla_actor.id,
            object_type,
            CarlaUtils.get_actor_bounding_size(carla_actor),
            CarlaUtils.vector3d_to_numpy(carla_actor.get_location()),
            CarlaUtils.vector3d_to_numpy(carla_actor.get_velocity()),
            CarlaUtils.get_actor_rotation_matrix(carla_actor),
            CarlaUtils.get_actor_angular_velocity(carla_actor),

            # Use stand-in values which assume complete certainty
            np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
            1.0
        )
