# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import json

import numpy as np
import yaml
import numpy
import os

from objects.DetectedObject import DetectedObject


class DetectedObjectEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, DetectedObject):

            dict_return = {
                'type': obj.type,
                'confidence': obj.confidence,
                'projString': obj.projString,
                'objectId': obj.objectId,
                'position':{'x': obj.position[0], 'y': obj.position[1], 'z': obj.position[2]},
                'positionCovariance': obj.positionCovariance.tolist(),
                'velocity':{'x': obj.velocity[0], 'y': obj.velocity[1], 'z': obj.velocity[2]},
                'velocityCovariance': obj.velocityCovariance.tolist(),
                'angularVelocity': {'x': obj.angularVelocity[0], 'y': obj.angularVelocity[1], 'z': obj.angularVelocity[2]},
                'angularVelocityCovariance': obj.angularVelocityCovariance.tolist(),
                'size': {'length': obj.size[0], 'height': obj.size[1], 'width': obj.size[2]},
                'timestamp': obj.timestamp,
                'sensorID': obj.sensorId
                #'bounding_box_in_world_coordinate_frame': [array.tolist() for array in obj.bounding_box_in_world_coordinate_frame],            
                #'rotation': obj.rotation.tolist(),        
                #'carla_actor': str(obj.carla_actor)
            }

            # Convert DetectedObject attributes to dictionary for serialization.
            # np.ndarray objects are converted to lists using the tolist() method.
            return dict_return
        # Handle numpy.ndarray objects
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        # Fallback to the base class default method for other types.
        return super(DetectedObjectEncoder, self).default(obj)


class SimulatedSensorUtils:
    """
    Generic utilities.
    """

    @staticmethod
    def load_config_from_file(config_filepath):
        """
        Load a yaml config file.
        :param config_filepath: Path to the config file.
        :return: Dictionary containing the configuration.
        """
        with open(config_filepath, "r") as file:
            config = yaml.safe_load(file)
            return config

    @staticmethod
    def serialize_to_json(obj):
        """
        Serialize any object to JSON. Specialized handling is provided for fields of type numpy.ndarray. Generalized
        deserialization is not possible.

        :param obj: Object to serialize.
        :return: JSON string.
        """

        if isinstance(obj, numpy.ndarray):
            return obj.tolist()
        elif isinstance(obj, list):
            data = [SimulatedSensorUtils.serialize_to_json(item) for item in obj]
            return "[" + str.join(",", data) + "]"
        else:
            return json.dumps(obj, cls=DetectedObjectEncoder)
