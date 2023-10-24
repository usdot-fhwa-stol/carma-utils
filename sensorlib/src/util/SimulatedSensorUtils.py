# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import json

import yaml
import numpy
from src.util.NumpyEncoder import NumpyEncoder
from src.objects.DetectedObject import DetectedObject

class DetectedObjectEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, DetectedObject):

            dict_return = {
                'id': obj.id,
                'object_type': obj.object_type,
                'timestamp': obj.timestamp,
                'bounding_box_in_world_coordinate_frame': [array.tolist() for array in obj.bounding_box_in_world_coordinate_frame],
                'position': obj.position.tolist(),
                'velocity': obj.velocity.tolist(),
                'rotation': obj.rotation.tolist(),
                'angular_velocity': obj.angular_velocity.tolist(),
                'position_covariance': obj.position_covariance.tolist(),
                'velocity_covariance': obj.velocity_covariance.tolist(),
                'confidence': obj.confidence,
                'carla_actor': str(obj.carla_actor)
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
            return json.dumps(data)
        else:
            return json.dumps(obj, cls=DetectedObjectEncoder)