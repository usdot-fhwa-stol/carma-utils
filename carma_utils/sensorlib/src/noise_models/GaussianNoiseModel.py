#!/usr/bin/python
# -*- coding: utf-8 -*-
import random

import numpy as np

from src.noise_models.AbstractNoiseModel import AbstractNoiseModel


class GaussianNoiseModel(AbstractNoiseModel):
    def __init__(self, config):
        self.__config = config
        self.__position_std = self.__config["noise_model_config"]["std_deviations"]["position"]
        self.__orientation_std = self.__config["noise_model_config"]["std_deviations"]["orientation"]
        self.__rng = np.random.default_rng()

    def apply_position_noise(self, object_list):
        # Apply position noise to the object_list
        noise_mean = 0.0
        noise_std = self.__position_std
        for obj in object_list:
            noise = np.random.normal(noise_mean, noise_std, size=3)
            obj.position += noise
        return object_list

    def apply_orientation_noise(self, object_list):
        # Apply orientation noise to the object_list
        noise_mean = 0.0
        noise_std = self.__orientation_std
        for obj in object_list:
            noise = np.random.normal(noise_mean, noise_std, size=(4, 4))
            obj.rotation += noise
        return object_list

    def apply_type_noise(self, object_list):
        # Apply type noise to the object_list
        for obj in object_list:
            obj.object_type = self.__rng.choice(self.__config["noise_model_config"]["type_noise"]["allowed_semantic_tags"], 1, replace=False)
        return object_list

    def apply_list_inclusion_noise(self, object_list):
        return object_list[0:np.random.randint(0, len(object_list) + 1)]
