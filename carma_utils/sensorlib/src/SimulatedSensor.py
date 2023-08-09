#!/usr/bin/python
# -*- coding: utf-8 -*-
from abc import abstractmethod

class SimulatedSensor:

    @abstractmethod
    def get_detected_objects_in_frame(self):
        pass
