from realsense import RealSenseHandler
import cv2
import pcl
import numpy as np

if __name__ == "__main__":
    handler = RealSenseHandler()

    try:
        while True:
            print("Fetching color image...")
            img = handler.get_img()
            if img is not None:
                print("Displaying color image...")
                cv2.imshow("RealSense Color Image", img)

            print("Fetching point cloud...")
            pointcloud = handler.get_pointcloud()
            if pointcloud is not None:
                print(f"PointCloud has {pointcloud.shape[0]} points.")
                pcl_pointcloud = pcl.PointCloud()
                pcl_pointcloud.from_array(np.array(pointcloud, dtype=np.float32))
                print("PointCloud loaded into PCL.")

            # Add a delay and exit condition to avoid infinite loop
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Exiting...")
                break

    finally:
        handler.stop()
        cv2.destroyAllWindows()
