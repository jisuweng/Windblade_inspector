#!/bin/bash

# Publish to the appropriate topic
rostopic pub /iris_0_px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
