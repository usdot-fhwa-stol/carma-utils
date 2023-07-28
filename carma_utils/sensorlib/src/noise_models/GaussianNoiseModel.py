#!/usr/bin/python
# -*- coding: utf-8 -*-
import random

import numpy as np

from src.noise_models.AbstractNoiseModel import AbstractNoiseModel


class GaussianNoiseModel(AbstractNoiseModel):
    def __init__(self, config):
        self.__config = config
        self.__position_std = self.config["noise_model_config"]["std_deviations"]["position"]
        self.__orientation_std = self.config["noise_model_config"]["std_deviations"]["orientation"]

    def apply_position_noise(self, object_list):
        # Apply position noise to the object_list
        noise_mean = self.config["position_noise"]["mean"]
        noise_std = self.config["position_noise"]["std"]
        for obj in object_list:
            noise = np.random.normal(noise_mean, noise_std, size=3)
            obj.position += noise
        return object_list

    def apply_orientation_noise(self, object_list):
        # Apply orientation noise to the object_list
        noise_mean = self.config["orientation_noise"]["mean"]
        noise_std = self.config["orientation_noise"]["std"]
        for obj in object_list:
            noise = np.random.normal(noise_mean, noise_std, size=(3, 3))
            obj.rotation = np.dot(noise, obj.rotation)
        return object_list

    def apply_type_noise(self, object_list):
        # Apply type noise to the object_list
        for obj in object_list:
            obj.object_type = random.sample(self.config["type_noise"]["possible_object_types"])
        return object_list

    def apply_list_inclusion_noise(self, object_list):
        return object_list[0:np.random.randint(0, len(object_list))]
