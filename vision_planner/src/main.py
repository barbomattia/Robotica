"""! @brief Everything regarding the Vision tasks and detection."""

##
# @mainpage Vision task
#
# @section description_main Description
# Explain the whole detection process: TODO
#
# @section notes_main Notes
# - Still a work in progress.
#
##
# @file detection.py
#
# @brief Performs block detection based on pre-trained ONNX model and OpenCV's Deep Neural Network submodule.
#
# @section description_doxygen_example Description
# Give overview: TODO.
#
# @section libraries_main Libraries/Modules
# - OpenCV (https://pypi.org/project/opencv-python/)
#   - Pre-process the image to perform inference on.
#   - Input the image to the network.
#   - Get outputs of neural network.
# - Numpy (https://numpy.org/)
#   - Manage network output to extrapolate useful data.

import open3d as o3d
import numpy as np
import detection as dc
import pose_detector as pd
import rospy as ros
import cv2 as cv
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2

class Vision:
    def __init__(self) -> None:
        self.image = None
        self.pcd = None
        self.results = []
        self.bridge = CvBridge()
        self.nn = dc.loadONNX()
    
    def convertRosToOpen3D(self):
        fields = [field.name for field in self.pcd.fields]
        data = pc2.read_points_list(self.pcd, fields, True)
        cloud = o3d.geometry.PointCloud()
        
        xyz = [(x, y, z) for x, y, z, _ in data]
        cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        
        self.pcd = cloud
    
    def imageCallback(self, _image: Image):
        try:
            print('Image acquired')
            image = self.bridge.imgmsg_to_cv2(_image, 'bgr8')
            self.image = image
            print('Bridged')
        except CvBridgeError as e:
            ros.logerr(e) 
    
    def pcdCallback(self, _pcd: PointCloud2):
        print('Point cloud acquired')
        self.pcd = _pcd
        self.convertRosToOpen3D()
        print('Point cloud converted')

    def process(self):
        detections = dc.inference(self.image, self.nn)
        print('Detected')
        dc.showBBox(self.image, detections)
        for detection in detections:
            self.results.append(f'{detection.className}: {detection.bbox}')
        print('Detections saved')
        self.pcd = pd.rotate_point_cloud(self.pcd)
        print('Point cloud rotated')
        intrinsics = o3d.camera.PinholeCameraIntrinsic(width=1920, height=1080, fx=790.94585984335442, fy=790.94585984335442, cx=960, cy=540)
        depth_frame = pd.create_depth_image_from_point_cloud(self.pcd, intrinsics)
        meshes = pd.load_meshes('./models/')
        print('Meshes loaded')
        blocks = pd.crop_depth_image(self.results, depth_frame, intrinsics)
        data = pd.find_best(meshes, blocks)
        for d in data:
            print(d)

if __name__ == '__main__':
    vision = Vision()
    
    ros.init_node('vision', anonymous=True)
    #ros.Subscriber('/ur5/zed_node/left/image_rect_color', Image, vision.imageCallback, queue_size = 1)
    #ros.Subscriber('/ur5/zed_node/point_cloud/cloud_registered', PointCloud2, vision.pcdCallback, queue_size = 1)
    
    vision.imageCallback(ros.wait_for_message('/ur5/zed_node/left/image_rect_color', Image))
    vision.pcdCallback(ros.wait_for_message('/ur5/zed_node/point_cloud/cloud_registered', PointCloud2))
    vision.process()
    
