# motion_predict

This library implements some basic motion prediction functions meant to support external object handling under the motion_predict namespace. Currently there are two motion models implements a Constant Velocity (CV) model and a Constant Turn-Rate and Velocity (CTRV) model. See the motion_predict.h and predict_ctrv.h files for the relevant interfaces.

> **Note:** Both motion models expect the objects' twist data to be in map frame as opposed to base_link frame commonly used in ROS.
For example:
pose.pose.x and pose.pose.y expected to be the location of the object in the map frame
twist.linear.x and twist.linear.y indicate the heading of the object in the map frame

This means that
pose.orientation is not meaningfully used except to pass it on.

See open issue: https://github.com/usdot-fhwa-stol/carma-platform/issues/2401

Link to detailed design on Confluence: https://usdot-carma.atlassian.net/l/c/e3U5DXZi
