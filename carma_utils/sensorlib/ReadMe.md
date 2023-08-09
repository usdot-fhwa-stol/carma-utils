# CARLA Sensor Library

## Overview

The sensor library provides a proxy sensor for the CARLA simulator, exposing additional features.

The SimulatedSensor acts as a wrapper to the CARLA Semantic Lidar Sensor, capturing the data and performing post-processing to generate a list of detected objects upon request.

## Context

This new feature is provided to enable vulnerable road user detection using a LIDAR sensor in the context of a CARLA-driven simulation. The data is intended to feed into a CARMA V2X system for data fusion with other sensors and action by CARMA platform vehicles.

## Description

The SimulatedSesnor manages a ProxySensor, which is a light wrapper around the CARLA sensor reference.

The SimulatedSesnsor.get_detected_objects_in_frame() function presents the primary processing sequence. Calls to internal utility classes build DetectedObject's, representing the detected vulnerable road users. High-speed LIDAR data is collected into a DataCollector class running in a separate context which self-populates via a callback.

The processing sequence performs the following stages:

1. Truth state retrieval.
1. Prefilter by distance from sensor.
1. LIDAR hitpoint retrieval.
1. Occlusion determination using a fractional hitpoint coverage threshold scaling proportionally with the sensor-to-object distance.
1. Application of a generic noise model to the results.

Processing frames are understood to consist of one full LIDAR sensor rotation, and may include vertical scan elements.

Configurability is provided to the SimulatedSensor, utility functions computing prefiltering and occlusion threshold scaling, and the noise model. Furthermore the noise model is extensible.
