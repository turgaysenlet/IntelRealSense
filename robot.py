# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2
import maestro
from inputs import devices
from inputs import get_gamepad

# -*- coding: utf-8 -*-
#from espeak import espeak
import os
    
# Configure depth and color streams
servo = maestro.Controller()
servo.setAccel(0,10)      #set servo 0 acceleration to 4
servo.setSpeed(0,10)     #set speed of servo 1

pipeline = rs.pipeline()
config = rs.config()
config.enable_device('746612070147')
# config.enable_stream(rs.stream.pose)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
print(config.can_resolve(pipeline))
for device in devices.gamepads:
    print(device)

# Start streaming
pipeline.start(config)
try:
    while True:
        # for event in events:
        # print('Joy events: ', event.ev_type, event.code, event.state)

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Get the depth frame's dimensions
        width = depth_frame.get_width()
        height = depth_frame.get_height()

        # Query the distance from the camera to the object in the center of the image
        d = []
        i = 1
        while i < 8:
            d = d + [depth_frame.get_distance(int(i * width / 8), int(height / 2))]
            i += 1

        dist_to_center = min(d)
        
        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        speed = 6000
        speed = speed + 200 * dist_to_center
        speed = min((6250, max((6000, speed))))
        print((speed, dist_to_center))
        servo.setTarget(0, int(speed))  #set servo to move to center position
        if dist_to_center < 0.5 and dist_to_center > 0:            
            servo.setAccel(0,0)       #set servo 0 acceleration to 4
            servo.setSpeed(0,0)       #set speed of servo 1
            servo.setTarget(0, 6000)  #set servo to move to center position
            os.system("espeak 'excuse me!'")
            servo.setAccel(0,10)      #set servo 0 acceleration to 4
            servo.setSpeed(0,10)     #set speed of servo 1
        cv2.waitKey(1)

finally:
    servo.setAccel(0,0)       #set servo 0 acceleration to 4
    servo.setSpeed(0,0)       #set speed of servo 1
    servo.setTarget(0, 6000)  #set servo to move to center position
    pipeline.stop()
    # Stop streaming
    servo.close()
