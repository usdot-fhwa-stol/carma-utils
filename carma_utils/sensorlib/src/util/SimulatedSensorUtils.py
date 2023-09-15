# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import json

import yaml

from src.util.NumpyEncoder import NumpyEncoder


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
        if isinstance(obj, list):
            data = [SimulatedSensorUtils.serialize_to_json(item) for item in obj]
            return json.dumps(data)
        else:
            return json.dumps(obj.__dict__, cls=NumpyEncoder)
