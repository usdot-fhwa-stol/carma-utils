class SimulatedSensorTestUtils:

    @staticmethod
    def generate_simulated_sensor_config():
        return {
            "prefilter": {
                "allowed_semantic_tags": ["Pedestrian", "Vehicle"],
                "max_distance_meters": 100
            },
            "detection_threshold_scaling_formula": {
                "nominal_hitpoint_detection_ratio_threshold": 0.6,
                "hitpoint_detection_ratio_threshold_per_meter_change_rate": -0.0033,
                "and_scaling_parameters_for_the_adjustable_threshold": {
                    "dropoff_rate": 0.01
                }
            }
        }

    @staticmethod
    def generate_test_data_sensed_objects():

        # Mock the carla.Actor class
        self.carla_actor = MagicMock()
        self.carla_actor.id = 1
        self.carla_actor.attributes = dict()
        self.carla_actor.is_alive = True
        self.carla_actor.parent = None
        self.carla_actor.semantic_types = ["Vehicles"]
        self.carla_actor.type_id = "vehicle.ford.mustang"

        self.carla_actor.get_acceleration = MagicMock(return_value=carla.Vector3D(0.0, 0.0, 0.0))
        self.carla_actor.get_angular_velocity = MagicMock(return_value=carla.Vector3D(0.0, 0.0, 0.005))
        self.carla_actor.get_location = MagicMock(return_value=carla.Location(10.0, 15.0, 7.0))
        self.carla_actor.get_transform = MagicMock(
            return_value=carla.Transform(carla.Location(10.0, 15.0, 7.0), carla.Rotation(3.0, 1.4, 4.0)))
        self.carla_actor.get_velocity = MagicMock(return_value=carla.Vector3D(100.0, 1.0, 0.0))
        self.carla_actor.get_world = MagicMock(return_value=carla.World)

        # Construct the SensedObject
        self.sensed_object = SensedObject(self.simulated_sensor_config, self.carla_actor)



        return [
            SensedObject(

            )
        ]

    @staticmethod
    def generate_test_data_hitpoints(self):
