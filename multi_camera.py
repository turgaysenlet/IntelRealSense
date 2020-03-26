import pyrealsense2 as rs
import numpy as np
import cv2
import logging


# # Configure depth and color streams...
# # ...from Camera 1
pipeline_1 = rs.pipeline()
config_1 = rs.config()
config_1.enable_device('746612070147')
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
# ...from Camera 2
pipeline_2 = rs.pipeline()
config_2 = rs.config()
# config_2.enable_device('943222110753')
config_2.enable_stream(rs.stream.pose)
config_2.enable_stream(rs.stream.fisheye, 1)
config_2.enable_stream(rs.stream.fisheye, 2)


# Start streaming from both cameras
print("start2...")
pipeline_2.start(config_2)
print("start2 done")
print("start1...")
pipeline_1.start(config_1)
print("start1 done")

try:
    while True:

        # Camera 1
        # Wait for a coherent pair of frames: depth and color
        frames_1 = pipeline_1.wait_for_frames()
        depth_frame_1 = frames_1.get_depth_frame()
        color_frame_1 = frames_1.get_color_frame()
        if not depth_frame_1 or not color_frame_1:
            continue
        # Convert images to numpy arrays
        depth_image_1 = np.asanyarray(depth_frame_1.get_data())
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap_1 = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_1, alpha=0.2), cv2.COLORMAP_JET)

        # Camera 2
        # Wait for a coherent pair of frames: depth and color
        frames_2 = pipeline_2.wait_for_frames()
        frameset = frames_2.as_frameset()
        fisheye_frame_1 = frameset.get_fisheye_frame(1).as_video_frame()
        fisheye_frame_2 = frameset.get_fisheye_frame(2).as_video_frame()

        # Convert images to numpy arrays
        fisheye_1 = np.asanyarray(fisheye_frame_1.get_data())
        fisheye_2 = np.asanyarray(fisheye_frame_2.get_data())
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        fisheye_color_1 = cv2.cvtColor(fisheye_1, cv2.COLOR_GRAY2BGR)
        fisheye_color_2 = cv2.cvtColor(fisheye_2, cv2.COLOR_GRAY2BGR)

        # fisheye_color_1 = cv2.applyColorMap(cv2.convertScaleAbs(fisheye_1, alpha=0.5), cv2.COLORMAP_JET)
        # fisheye_color_2 = cv2.applyColorMap(cv2.convertScaleAbs(fisheye_2, alpha=0.5), cv2.COLORMAP_JET)
        fisheye_color_resized_1 = cv2.resize(fisheye_color_1, (640, 480))
        fisheye_color_resized_2 = cv2.resize(fisheye_color_2, (640, 480))

        # Stack all images horizontally
        images = np.vstack((np.hstack((color_image_1, depth_colormap_1)), np.hstack((fisheye_color_resized_1, fisheye_color_resized_2))))

        # Show images from both cameras
        cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)
        cv2.imshow('RealSense', images)

        # Save images and depth maps from both cameras by pressing 's'
        ch = cv2.waitKey(25)
        if ch==115:
            cv2.imwrite("color.jpg", color_image_1)
            cv2.imwrite("depth.jpg", depth_colormap_1)
            cv2.imwrite("fisheye1.jpg", fisheye_color_1)
            cv2.imwrite("fisheye2.jpg", fisheye_color_2)
            print("Save")
        # Press q or Q to quit
        if ch==113 or ch==81:
            break

finally:

    # Stop streaming
    pipeline_1.stop()
    pipeline_2.stop()