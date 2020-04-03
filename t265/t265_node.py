import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, TransformStamped
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs
# import tf2_ros
# from tf2_ros.transform_broadcaster import TransformBroadcaster
import numpy as np
import cv2
import logging
import sys


class T265(Node):
    DEFAULT_ODOM_FRAME_ID = "odom_frame"
    DEFAULT_POSE_OPTICAL_FRAME_ID = "camera_pose_optical_frame"
    
    def __init__(self):
        super().__init__('T265')
        print('T265...')
        self.init_cam()
        print("stop...")
        # self.pipeline_2.stop()
        print("stop.")
        sys.exit()

        # self.init_camera()
        # self.resolve()
        # self.start_pipeline()
        # self.sub = self.create_subscription(String, 'chatter', self.chatter_callback, 10)        
        print('T265.')

    def init_cam(self):
        self.pipeline_2 = rs.pipeline()
        self.config_2 = rs.config()
        self.config_2.enable_stream(rs.stream.pose)
        # self.config_2.enable_stream(rs.stream.fisheye, 1)
        # self.config_2.enable_stream(rs.stream.fisheye, 2)
        print(self.config_2.can_resolve(self.pipeline_2))
        if self.config_2.can_resolve(self.pipeline_2) == False:
            self.resolved = False
            print('T265 resolve cannot resolve config!')
            sys.exit()
        else:
            self.resolved = True
        print("start...")
        if self.resolved == False:
            print('T265 not resolved skipping start!')
        else:                
            self.pipeline_2.start(self.config_2)
        print("start.")
        self.setup_publishers()
        self.publish_images()
        
    def __del__(self):
        print('T265 del...')
        self.destroy_camera()
        print('T265 del.')

    def destroy_camera(self):
        print('T265 destroy_camera...')
        if self.pipeline_2:
            print('T265 destroy_camera pipeline_2_...')
            if self.resolved == False:
                print('T265 not resolved skipping stop!')
            else:
                self.pipeline_2.stop()
            print('T265 destroy_camera pipeline_2_.')
        print('T265 destroy_camera.')
    
    def setup_publishers(self):
        print('T265 setup_publishers...')
        self.pub1 = self.create_publisher(Image, '/t265/fisheye1/image_raw', 2)
        self.pub2 = self.create_publisher(Image, '/t265/fisheye2/image_raw', 2)
        self.bridge = CvBridge()        
        print('T265 setup_publishers.')
    
    def publish_pose(self, frames):
        pose_frame = frames.get_pose_frame()
        if not pose_frame:
            return pose_frame
        pose = pose_frame.get_pose_data()
        typed = frames.get_profile().stream_type()
        index = frames.get_profile().stream_index()
        print(typed)
        print(index)
        print(pose.translation)

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = -pose.translation.z
        pose_msg.pose.position.y = -pose.translation.x
        pose_msg.pose.position.z = pose.translation.y
        pose_msg.pose.orientation.x = -pose.rotation.z
        pose_msg.pose.orientation.y = -pose.rotation.x
        pose_msg.pose.orientation.z = pose.rotation.y
        pose_msg.pose.orientation.w = pose.rotation.w
        # br = tf2_ros.TransformBroadcaster()
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.DEFAULT_ODOM_FRAME_ID
        msg.child_frame_id = self.DEFAULT_POSE_OPTICAL_FRAME_ID
        msg.transform.translation.x = pose_msg.pose.position.x
        msg.transform.translation.y = pose_msg.pose.position.y
        msg.transform.translation.z = pose_msg.pose.position.z
        msg.transform.rotation.x = pose_msg.pose.orientation.x
        msg.transform.rotation.y = pose_msg.pose.orientation.y
        msg.transform.rotation.z = pose_msg.pose.orientation.z
        msg.transform.rotation.w = pose_msg.pose.orientation.w

        # br.sendTransform(msg)
        return pose_frame

    def publish_images(self):
        print('T265 publish_images...')
        try:
            while True:
                # Camera 2
                # Wait for a coherent pair of frames: depth and color
                # frames_2 = self.pipeline_2.wait_for_frames()
                # Fetch pose frame
                frames = self.pipeline_2.wait_for_frames()                
                pose_frame = self.publish_pose(frames)
                if not pose_frame:
                    frames = self.pipeline_2.wait_for_frames()
                    frameset = frames.as_frameset()
                    fisheye_frame_1 = frameset.get_fisheye_frame(1).as_video_frame()
                    fisheye_frame_2 = frameset.get_fisheye_frame(2).as_video_frame()

                    # Convert images to numpy arrays
                    fisheye_1 = np.asanyarray(fisheye_frame_1.get_data())
                    fisheye_2 = np.asanyarray(fisheye_frame_2.get_data())
                    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                    fisheye_color_1 = cv2.cvtColor(fisheye_1, cv2.COLOR_GRAY2BGR)
                    fisheye_color_2 = cv2.cvtColor(fisheye_2, cv2.COLOR_GRAY2BGR)

                    self.pub1.publish(self.bridge.cv2_to_imgmsg(fisheye_color_1, "bgr8"))
                    self.pub2.publish(self.bridge.cv2_to_imgmsg(fisheye_color_2, "bgr8"))

        finally:
            print('T265 while ended.')
        print('T265 publish_images.')

def main(args=None):
    print('main...')

    # pipeline_2 = rs.pipeline()
    # config_2 = rs.config()
    # config_2.enable_stream(rs.stream.pose)
    # config_2.enable_stream(rs.stream.fisheye, 1)
    # config_2.enable_stream(rs.stream.fisheye, 2)
    # print(config_2.can_resolve(pipeline_2))
    # print("start...")
    # pipeline_2.start(config_2)
    # print("start.")
    # print("stop...")
    # pipeline_2.stop()
    # print("stop.")
    # sys.exit()
    rclpy.init(args=args)

    node = T265()
    try:
        print('main spin...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('main keyboard exception.')
        pass
    print('main spin.')
    print('main destroy...')
    node.destroy_node()
    print('main destroy.')
    print('main shutdown...')
    rclpy.shutdown()
    print('main shutdown.')
    print('main.')


if __name__ == '__main__':
    main()
