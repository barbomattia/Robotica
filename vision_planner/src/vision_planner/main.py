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
import vision_planner.detection as dc
import vision_planner.pose_detector as pd
import rospy as ros
import rospkg
import os
import cv2 as cv
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from vision_planner.srv import objectDetection, objectDetectionResponse

class Vision:
    def __init__(self) -> None:
        self.image = None
        self.pcd = None
        self.results = []
        self.bridge = CvBridge()
        self.nn = dc.loadONNX()
        ros.loginfo('Opened the neural network')
    
    def convertRosToOpen3D(self):
        fields = [field.name for field in self.pcd.fields]
        data = pc2.read_points_list(self.pcd, fields, True)
        cloud = o3d.geometry.PointCloud()
        
        xyz = [(x, y, z) for x, y, z, _ in data]
        cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        
        self.pcd = cloud
    
    def imageCallback(self, _image: Image):
        try:
            ros.loginfo('Image acquired')
            image = self.bridge.imgmsg_to_cv2(_image, 'bgr8')
            self.image = image
            ros.loginfo('Bridged to Mat')
        except CvBridgeError as e:
            ros.logfatal(e) 
    
    def pcdCallback(self, _pcd: PointCloud2):
        ros.loginfo('Point cloud acquired')
        self.pcd = _pcd
        self.convertRosToOpen3D()
        ros.loginfo('Point cloud converted')

    def process(self, req):
        self.imageCallback(ros.wait_for_message('/ur5/zed_node/left/image_rect_color', Image))
        self.pcdCallback(ros.wait_for_message('/ur5/zed_node/point_cloud/cloud_registered', PointCloud2))
        
        detections = dc.inference(self.image, self.nn)
        ros.loginfo('Blocks detected')
        dc.showBBox(self.image, detections)
        
        detections = sorted(detections, key = lambda x:x.confidence, reverse = True)
        for detection in detections:
            self.results.append(f'{detection.className}: {detection.bbox}')
        ros.loginfo('Block detections were saved')
        
        self.pcd = pd.rotate_point_cloud(self.pcd)
        ros.loginfo('Point cloud was rotated')
        intrinsics = o3d.camera.PinholeCameraIntrinsic(width=1920, height=1080, fx=790.94585984335442, fy=790.94585984335442, cx=960, cy=540)
        depth_frame = pd.create_depth_image_from_point_cloud(self.pcd, intrinsics)
        
        vision_path = rospkg.RosPack().get_path('vision_planner')
        meshes = pd.load_meshes(os.path.join(vision_path, 'models'))
        ros.loginfo('Meshes are loaded')
        
        blocks = pd.crop_depth_image(self.results, depth_frame, intrinsics)
        ros.loginfo('Cropped the blocks')
        data = pd.find_best(meshes, blocks)
        ros.loginfo('Pose and center extracted')
        
        nameBlocks = []
        xBlocks = []
        phiBlocks = []
        for block in data:
            ros.loginfo(block)
            nameBlocks.append(block[0])
            for pos in block[1]:
                xBlocks.append(pos)
            for angle in block[2]:
                phiBlocks.append(angle)
            
        response = objectDetectionResponse()
        response.xBlocks = xBlocks
        response.phiBlocks = phiBlocks
        response.nameBlocks = nameBlocks
        
        ros.loginfo('Sending detection data to task planner')
        return response

if __name__ == '__main__':
    vision = Vision()
    
    ros.init_node('vision', anonymous=True)
    ros.Service('object_detection', objectDetection, vision.process)
    ros.loginfo('Block Detection service is ready to process.')
    ros.spin()
