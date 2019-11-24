#!/bin/bash

rosbag record -o everything /imu/data /baro /status /GroundStation/vicon/filtered /aerowake_1/vicon/filtered /camera/image_raw /camera/camera_info
