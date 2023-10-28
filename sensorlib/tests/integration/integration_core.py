
class CarlaCDASimIntegrationTestCore:

    def __init__(self):
        self.carla_world = None
        self.is_visualization_active = False

    def build_world(self):
        pass

    def set_map(self):
        pass

    def add_scene_objects(self):
        pass

    def activate_overview_visualization(self):
        pass

    def activate_lidar_visualization(self, carla_lidar_sensor_actor_id):
        # Find sensor
        # Register callback
        pass


