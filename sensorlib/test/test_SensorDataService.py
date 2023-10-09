# Copyright (C) 2023 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
# the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
# applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.

import unittest
from unittest.mock import MagicMock


class TestService(unittest.TestCase):
    def test-start_xml_rpc_server(self):

        SimulatedSensorConfigurator.register_simulated_semantic_lidar_sensor = MagicMock(return_value=0)

        # TODO


        self.assertEqual(main(), "Hello World!")

    def test_create_simulated_semantic_lidar_sensor(self):
        assert False

    def test_get_simulated_sensor(self):
        assert False

    def test_get_detected_objects(self):
        assert False
        detected_objects = SimulatedSensorTestUtils.generate_test_data_detected_objects()
        detected_objects = [replace(obj, carla_actor=None) for obj in detected_objects]
        self.sensor._SemanticLidarSensor__detected_objects = detected_objects
        serialized = self.sensor.get_detected_objects_json()
        with open("data/test_data_serialized_detected_objects.json", "r") as file:
            expected_serialized_data = json.load(file)
            assert serialized == expected_serialized_data
        # TODO Remove
        # with open("data/test_data_serialized_detected_objects.json", "w") as file:
        #     json.dump(serialized, file)




