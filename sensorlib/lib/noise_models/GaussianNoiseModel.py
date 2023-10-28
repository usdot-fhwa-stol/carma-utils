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
        self.__position_std = self.__config["std_deviations"]["position"]
        self.__orientation_std = self.__config["std_deviations"]["orientation"]
        self.__rng = np.random.default_rng()

    def apply_position_noise(self, object_list):

        if not self.__config["stages"]["position_noise"]:
            return object_list

        # Apply position noise to the object_list
        noise_mean = 0.0
        noise_std = self.__position_std
        for obj in object_list:
            noise = np.random.normal(noise_mean, noise_std, size=3)
            obj.position[0] += noise[0]
            obj.position[1] += noise[1]
            obj.position[2] += noise[2]

        return object_list

    def apply_orientation_noise(self, object_list):

        if not self.__config["stages"]["orientation_noise"]:
            return object_list

        # Apply orientation noise to the object_list
        noise_mean = 0.0
        noise_std = self.__orientation_std
        for obj in object_list:
            noise = np.random.normal(noise_mean, noise_std, size=3)
            obj.rotation[0] += noise[0]
            obj.rotation[1] += noise[1]
            obj.rotation[2] += noise[2]

        return object_list

    def apply_type_noise(self, object_list):
        if not self.__config["stages"]["type_noise"]:
            return object_list

        # Apply type noise to the object_list
        return [replace(obj,
                        object_type=CarlaUtils.get_semantic_tag_name(
                            self.__rng.choice(self.__config["type_noise"]["allowed_semantic_tags"],
                                                      1, replace=False)))
                for obj in object_list]

    def apply_list_inclusion_noise(self, object_list):
        if not self.__config["stages"]["list_inclusion_noise"]:
            return object_list
        return object_list[0:np.random.randint(0, len(object_list) + 1)]
