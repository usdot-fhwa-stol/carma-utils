# CARLA Sensor Library

## Overview

The sensor library provides a proxy sensor for the CARLA simulator, exposing additional features. 

The SimulatedSensor acts as a wrapper to the CARLA Semantic Lidar Sensor, capturing the data and performing post-processing to generate a list of detected objects upon request.

Raw sensor observations from the CARLA sensor are composed to build a dataset representing the current objects in scene. 

## Contexts

This new feature is provided to enable vulnerable road user detection using a LIDAR sensor in the context of a CARLA-driven simulation. The data is intended to feed into a CARMA V2X system for data fusion with other sensors and action by CARMA platform vehicles.

## Operation

The SimulatedSensor manages a CarlaSensor, which is a light wrapper around the CARLA sensor reference.

High-speed LIDAR data is collected into a DataCollector class running in a separate context which self-populates via a callback.

The SimulatedSensor.compute_detected_objects() function presents the primary processing sequence performing the following stages:

1. Truth state retrieval.
1. Pre-filter by distance from sensor, and allowed object types.
1. LIDAR hitpoint retrieval.
1. Occlusion filtering.
1. Application of a generic noise model to the results.

Internal functions build a list of DetectedObjects, representing the detected vulnerable road users. Range and angular extent are collected in lookup structures which are passed between functions as needed.

## Configuration

Configurability is provided to the SimulatedSensor including parameters related to: Pre-filtering, occlusion, and the noise model. Configuration parameters are required in the SemanticLidarSensor constructor, and available to load from included configuration files.

## Truth State Retrieval

The CARLA World.get_actors() function exposes actors in the simulation. These are immediately transformed into DetectedObject instances.

## Pre-Filtering

Objects not of the permitted type are removed, along with objects outside of the maximum range from the sensor.

## LIDAR Hitpoint Retrieval

Data are collected via the SensorDataCollector.__collect_sensor_data() function, which is registered as a callback in the CARLA sensor. Processing frames are collected into "active" and "previous" queue positions, are understood to consist of one full LIDAR sensor rotation. Data contains vertical scan elements.

## Occlusion Filtering

Objects which are occluded by other items in the scene are filtered out.

Occlusion is determined from a fractional threshold pertaining to the number of LIDAR hitpoints detected divided by the number expected.

To account for innacuracies in these calculations due to beam spreading at larger distances, a scaling factor is applied to the threshold. The nominal threshold and scaling parameters are configurable.

## Noise Model

The noise model applies adjustments to the output data including positions, orientations, and whether the object has been detected.

The noise model interface is extensible and can support additional models in the future.
