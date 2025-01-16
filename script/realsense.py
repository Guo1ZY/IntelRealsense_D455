import pyrealsense2 as rs
import numpy as np
import cv2
from settings import RealSenseSettings


class RealSenseHandler:
    def __init__(self):
        self.settings = RealSenseSettings()
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        color_config = self.settings.get_color_stream_config()
        depth_config = self.settings.get_depth_stream_config()

        self.config.enable_stream(
            rs.stream.color,
            color_config["width"],
            color_config["height"],
            rs.format.bgr8,
            color_config["framerate"],
        )
        self.config.enable_stream(
            rs.stream.depth,
            depth_config["width"],
            depth_config["height"],
            rs.format.z16,
            depth_config["framerate"],
        )

        self.pipeline.start(self.config)

    def get_img(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    def get_pointcloud(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None

        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)
        vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        return vertices

    def stop(self):
        self.pipeline.stop()
