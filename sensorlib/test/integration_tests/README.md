# `sensorlib` integration tests

This directory contains various integration tests for the `sensorlib` library.

# Field of view filtering

This test spawns two semantic lidar sensors. One if attached to a traffic light
at an intersection, and the other is attached to a vehicle crossing the
intersection. This test relies on the `SensorlibLidarFieldOfView_1`
ScenarioRunner scenario (see the scenario's
[definition][scenario_definition_link] and
[configuration][scenario_configuration_link]).

To run this integration test:

1. Launch CARLA
2. Run the `SensorlibLidarFieldOfView_1` scenario
3. Run the `filed_of_view_filtering.py` Python script in this directory

> [!IMPORTANT]\
> You should run the integration test script as soon as you see the
> ScenarioRunner actors spawn.

[scenario_definition_link]: https://github.com/usdot-fhwa-stol/cdasim/blob/develop/scenario-runner/examples/sensorlib_lidar_field_of_view.py
[scenario_configuration_link]: https://github.com/usdot-fhwa-stol/cdasim/blob/develop/scenario-runner/examples/sensorlib_lidar_field_of_view.xml
