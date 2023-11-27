# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

from dataclasses import replace

import numpy as np

from noise_models.AbstractNoiseModel import AbstractNoiseModel
from util.CarlaUtils import CarlaUtils


class GaussianNoiseModel(AbstractNoiseModel):
    """Noise model which uses a generally Gaussian distribution for noise application."""

    def __init__(self, config):
        self.__config = config
        self.__position_std_in_meters = self.__config["std_deviations"]["position_in_meters"]
        self.__orientation_std_in_radians = self.__config["std_deviations"]["orientation_in_radians"]
        self.__linear_velocity_in_ms = self.__config["std_deviations"]["linear_velocity_in_ms"]
        self.__angular_velocity_in_rs = self.__config["std_deviations"]["angular_velocity_in_rs"]
        self.__rng = np.random.default_rng()

    def apply_position_noise(self, object_list):

        if not self.__config["stages"]["position_noise"]:
            return object_list

        # Apply position noise to the object_list
        noise_mean = 0.0
        for obj in object_list:
            noise = np.random.normal(noise_mean, self.__position_std_in_meters, size=3)
            obj.position[0] += noise[0]
            obj.position[1] += noise[1]
            obj.position[2] += noise[2]

        return object_list

    def apply_orientation_noise(self, object_list):

        if not self.__config["stages"]["orientation_noise"]:
            return object_list

        # Apply orientation noise to the object_list
        noise_mean = 0.0
        for obj in object_list:
            noise = np.random.normal(noise_mean, self.__orientation_std_in_radians, size=3)

            obj.rotation[0] += noise[0]
            obj.rotation[1] += noise[1]
            obj.rotation[2] += noise[2]

        return object_list

    def apply_type_noise(self, object_list):
        if not self.__config["stages"]["type_noise"]:
            return object_list

        # Apply type noise to the object_list
        return [replace(obj,
                        type= self.__rng.choice(self.__config["type_noise"]["allowed_semantic_tags"], 1))
                for obj in object_list]

    def apply_list_inclusion_noise(self, object_list):
        if not self.__config["stages"]["list_inclusion_noise"]:
            return object_list
        return object_list[0:np.random.randint(0, len(object_list) + 1)]

    def apply_position_covariance_noise(self, object_list):
        if not self.__config["stages"]["position_noise"]:
            return object_list

        # Apply position noise to the object_list
        for obj in object_list:
            obj.positionCovariance[0][0] = self.__position_std_in_meters[0] ** 2
            obj.positionCovariance[1][1] = self.__position_std_in_meters[1] ** 2
            obj.positionCovariance[2][2] = self.__position_std_in_meters[2] ** 2

        return object_list

    def apply_orientation_covariance_noise(self, object_list):
        if not self.__config["stages"]["orientation_noise"]:
            return object_list

        # Apply orientation noise to the object_list
        for obj in object_list:
            obj.orientationCovariance[0][0] = self.__orientation_std_in_radians[0] ** 2
            obj.orientationCovariance[1][1] = self.__orientation_std_in_radians[1] ** 2
            obj.orientationCovariance[2][2] = self.__orientation_std_in_radians[2] ** 2

        return object_list

    def apply_linear_velocity_covariance_noise(self, object_list):
        if not self.__config["stages"]["linear_velocity_noise"]:
            return object_list

        # Apply velocity noise to the object_list
        for obj in object_list:
            obj.velocityCovariance[0][0] = self.__linear_velocity_in_ms[0] ** 2
            obj.velocityCovariance[1][1] = self.__linear_velocity_in_ms[1] ** 2
            obj.velocityCovariance[2][2] = self.__linear_velocity_in_ms[2] ** 2

        return object_list

    def apply_angular_velocity_covariance_noise(self, object_list):
        if not self.__config["stages"]["angular_velocity_noise"]:
            return object_list

        # Apply angular velocity noise to the object_list
        for obj in object_list:
            obj.angularVelocityCovariance[0][0] = self.__angular_velocity_in_rs[0] ** 2
            obj.angularVelocityCovariance[1][1] = self.__angular_velocity_in_rs[1] ** 2
            obj.angularVelocityCovariance[2][2] = self.__angular_velocity_in_rs[2] ** 2

        return object_list