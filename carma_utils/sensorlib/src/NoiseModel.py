#!/usr/bin/python
#-*- coding: utf-8 -*-
import numpy as np


class NoiseModel:
    def __init__(self, config):
        self.__config = config
        self.__position_std = self.config["noise_model_config"]["std_deviations"]["position"]
        self.__orientation_std = self.config["noise_model_config"]["std_deviations"]["orientation"]

    def apply_position_noise(self, object_list):
        # Apply position noise to the object_list
        # Example implementation:
        noise_mean = self.config["position_noise"]["mean"]
        noise_std = self.config["position_noise"]["std"]
        for obj in object_list:
            noise = np.random.normal(noise_mean, noise_std, size=3)
            obj.position += noise
        return object_list

    def apply_orientation_noise(self, object_list):
        # Apply orientation noise to the object_list
        # Example implementation:
        noise_mean = self.config["orientation_noise"]["mean"]
        noise_std = self.config["orientation_noise"]["std"]
        for obj in object_list:
            noise = np.random.normal(noise_mean, noise_std, size=(3, 3))
            obj.rotation = np.dot(noise, obj.rotation)
        return object_list

    def apply_type_noise(self, object_list):
        # Apply type noise to the object_list
        # Example implementation:
        for obj in object_list:
            obj.object_type = "Unknown"
        return object_list

    def apply_list_inclusion_noise(self, object_list, excluded_object_list):
        # Apply list inclusion noise to the object_list
        # Example implementation:
        inclusion_prob = self.config["list_inclusion_noise"]["inclusion_prob"]
        for obj in excluded_object_list:
            if np.random.uniform() < inclusion_prob:
                object_list.append(obj)
        return object_list