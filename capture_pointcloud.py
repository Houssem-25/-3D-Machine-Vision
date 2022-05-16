import numpy as np
import cv2
import open3d as o3d
import argparse
import sys, getopt


def parse_inputs():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output_file', type=str, default='out.ply', help='path of the output point cloud file')
    parser.add_argument('--timer', type=int, default=3, help='number of acquisitions')
    parser.add_argument('--device', type=int, default=0, help='id of the used device')

    return parser.parse_args()


class AzureKinect:
    """
    class for azure kinect related functions
    """
    def __init__(self):
        self.config = o3d.io.read_azure_kinect_sensor_config('default_config.json')
        self.device = 0
        self.align_depth_to_color = 1

    def start(self):
        self.sensor = o3d.io.AzureKinectSensor(self.config)
        if not self.sensor.connect(self.device):
            raise RuntimeError('Failed to connect to sensor')

    def frames(self):
        while 1:
            rgbd = self.sensor.capture_frame(self.align_depth_to_color)
            if rgbd is None:
                continue
            color,depth = np.asarray(rgbd.color).astype(np.uint8),np.asarray(rgbd.depth).astype(np.float32) / 1000.0
            return color, depth

class Create_point_cloud():
    """
    Class contain functions to generate and save point clouds
    """
    def __init__(self, parameters):
        self.count = 0
        self.timer = parameters.timer
        self.outputfile = parameters.output_file
        self.cam = AzureKinect()
        self.cam.start()


    def create_point_cloud(self):
        """
        this function generate a ply file containing point clouds

        """
        while self.count < self.timer:
            self.count += 1
            color_frame, depth_frame = self.cam.frames()

            depth_image=(depth_frame*1000).astype(np.uint16)
            color_image=color_frame
            # color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            img_depth = o3d.geometry.Image(depth_image)
            img_color = o3d.geometry.Image(color_image)
            # generate RGBD image by combining depth and rgb frames
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth, convert_rgb_to_intensity=False,
                                                                      depth_trunc=15.0)

            # default camera intrinsic parameters for azure kinect sensor
            intrinsic = o3d.camera.PinholeCameraIntrinsic(1280, 720, 608.9779052734375, 608.77679443359375,
                                                          636.66748046875,
                                                          367.427490234375)

            #generate point cloud from RGBD
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

            #display point cloud
            o3d.visualization.draw_geometries([pcd])

            # transform the point cloud
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

            # display transformed point cloud
            o3d.visualization.draw_geometries([pcd])

            #display color image
            cv2.imshow('bgr', cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(1)
            if key == ord('q'):
                break

        #write pointcloud in ply file
        o3d.io.write_point_cloud(self.outputfile, pcd)
        cv2.destroyAllWindows()


if __name__=="__main__":
    # create object of point cloud class
    parameters = parse_inputs()
    pc=Create_point_cloud(parameters)
    pc.create_point_cloud()
