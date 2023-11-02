import os
import unittest
from abc import abstractmethod

import carla
from carla_cda_sim_api import CarlaCDASimAPI


class SensorlibIntegrationTestRunner(unittest.TestCase):

    def setUp(self):
        carla_host = os.getenv('CARLA_HOST', 'localhost')
        carla_port = int(os.getenv('CARLA_PORT', '2000'))
        self.carla_world = self.get_carla_connection(carla_host, carla_port)
        self.set_map(self.carla_world, "Town04")
        self.setup_scenario(self.carla_world)
        self.api = self.build_api_object(self.carla_world)

    def get_carla_connection(self, carla_host, carla_port):
        client = carla.Client(carla_host, carla_port)
        return client.get_world()


    def set_map(self, carla_client, map_name):
        # TODO
        carla_client.set_map(map_name)
        self.world.set_weather(carla.WeatherParameters.ClearNoon)

    def build_api_object(self, carla_world):
        return CarlaCDASimAPI.build_from_world(carla_world)

    @abstractmethod
    def setup_scenario(self):
        pass
