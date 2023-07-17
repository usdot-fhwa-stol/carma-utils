#!/usr/bin/python
#-*- coding: utf-8 -*-

class NoiseModel:
    def __init__(self):
        self.config = None

    def load_config(self, ):
        pass

    def apply_numeric_noise(self, x):
        pass

    def apply_position_noise(self, object_list):
        pass

    def apply_orientation_noise(self, object_list):
        pass

    def apply_type_noise(self, object_list):
        pass

    def apply_list_inclusion_noise(self, object_list, excluded_object_list):
        pass

