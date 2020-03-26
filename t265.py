#!/usr/bin/python
# -*- coding: utf-8 -*-
# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#####################################################
##           librealsense T265 example             ##
#####################################################

import pyrealsense2 as rs
import numpy as np
import cv2

# DOES NOT WORK config.enable_device('943222110753')
# DOES NOT WORK config.enable_device(serial_number)
# DOES NOT WORK config.enable_stream(rs.stream.pose, rs.format.six_dof)
# DOES NOT WORK ctx = rs.context() -> works
# DOES NOT WORK sensors = ctx.query_all_sensors()
# DOES NOT WORK pipeline = rs.pipeline(ctx)

# WORKS config.enable_stream(rs.stream.pose, rs.format.six_dof, 200) -> not with fisheye 
# WORKS config.enable_stream(rs.stream.pose)
# WORKS config.enable_stream(rs.stream.fisheye, 1)
# WORKS config.enable_stream(rs.stream.fisheye, 2)

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipeline = rs.pipeline()

# Build config object and request pose data
config = rs.config()
config.enable_stream(rs.stream.pose)
config.enable_stream(rs.stream.fisheye, 1)
config.enable_stream(rs.stream.fisheye, 2)
print(config.can_resolve(pipeline))

# Start streaming with requested config
print("starting...")
pipeline.start(config)
print("started.")

try:
    for _ in range(50):
        # Wait for the next set of frames from the camera
        frames = pipeline.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()
            print("Frame #{}".format(pose.frame_number))
            print("Position: {}".format(data.translation))
            print("Velocity: {}".format(data.velocity))
            print("Acceleration: {}\n".format(data.acceleration))
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())


finally:
    pipeline.stop()
