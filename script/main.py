from realsense import RealSenseHandler
import cv2
import open3d as o3d
import numpy as np

# 时间
import time

if __name__ == "__main__":
    handler = RealSenseHandler()

    try:
        vis = o3d.visualization.Visualizer()
        vis.create_window("PointCloud Viewer")
        pointcloud_o3d = o3d.geometry.PointCloud()
        vis.add_geometry(pointcloud_o3d)

        while True:
            # print("Fetching color image...")
            # time
            start = time.time()
            # img = handler.get_img()
            # if img is not None:
            #     # print("Displaying color image...")
            #     cv2.imshow("RealSense Color Image", img)
            # time
            # end = time.time()
            # # timeuse in ms
            # print("timeuse:", (end - start) * 1000)
            # print("Fetching point cloud...")
            pointcloud = handler.get_pointcloud()
            if pointcloud is not None:
                print(f"PointCloud has {pointcloud.shape[0]} points.")
                pointcloud_o3d.points = o3d.utility.Vector3dVector(pointcloud)
                vis.update_geometry(pointcloud_o3d)
                vis.poll_events()
                vis.update_renderer()
            end = time.time()
            # timeuse in ms
            print("timeuse:", (end - start) * 1000)
            # Add a delay and exit condition to avoid infinite loop
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Exiting...")
                break

    finally:
        handler.stop()
        vis.destroy_window()
        cv2.destroyAllWindows()
