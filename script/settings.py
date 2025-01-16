class RealSenseSettings:
    def __init__(self):
        self.color_resolution = (640, 360)  # (width, height)
        self.depth_resolution = (640, 480)  # (width, height)
        self.color_framerate = 90  # frames per second
        self.depth_framerate = 90  # frames per second

    def get_color_stream_config(self):
        return {
            "width": self.color_resolution[0],
            "height": self.color_resolution[1],
            "framerate": self.color_framerate,
        }

    def get_depth_stream_config(self):
        return {
            "width": self.depth_resolution[0],
            "height": self.depth_resolution[1],
            "framerate": self.depth_framerate,
        }

    def __str__(self):
        color_config = f"{self.color_resolution[0]}x{self.color_resolution[1]} @{self.color_framerate}fps"
        depth_config = f"{self.depth_resolution[0]}x{self.depth_resolution[1]} @{self.depth_framerate}fps"
        return f"Color : {color_config}\nDepth : {depth_config}"


# Example usage:
if __name__ == "__main__":
    settings = RealSenseSettings()
    print(settings)
