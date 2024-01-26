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
import rospy as ros
import cv2 as cv
import sensor_msgs.point_cloud2 as pc2
from threading import Lock as lk
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField

class Vision:
    def __init__(self) -> None:
        self.image = None
        self.lock  = lk()
        self.pcl = None
        self.bridge = CvBridge()
        self.nn = dc.loadONNX()
    
    def convertRosToOpen3D(self):
        fields = [field.name for field in self.pcl.fields]
        data = pc2.read_points_list(self.pcl, fields, True)
        cloud = o3d.geometry.PointCloud()
        
        xyz = [(x, y, z) for x, y, z, _ in data]
        cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        
        return cloud
        
    
    
    def imageCallback(self, _image: Image):
        with self.lock:
            try:
                print('Callback')
                image = self.bridge.imgmsg_to_cv2(_image, 'bgr8')
                self.image = image
                print('Bridged')
                detections = dc.inference(self.image, self.nn)
                print('Detected')
                dc.showBBox(self.image, detections)
                with open('detections.txt', 'w') as file:
                    for detection in detections:
                        file.write(f'{detection.className}: {detection.bbox}\n')
                        print(f'Class: {detection.className} // BBox: {detection.bbox}')
            except CvBridgeError as e:
                ros.logerr(e) 
    
    def pclCallback(self, _pcl: PointCloud2):
        with self.lock:
            self.pcl = _pcl
            self.pcl = self.convertRosToOpen3D()
            print(self.pcl)
            o3d.io.write_point_cloud('mostro.ply', self.pcl)
            

if __name__ == '__main__':
    vision = Vision()
    
    ros.init_node('image_converter', anonymous=True)
    ros.Subscriber('/ur5/zed_node/left/image_rect_color', Image, vision.imageCallback, queue_size = 1)
    ros.Subscriber('/ur5/zed_node/point_cloud/cloud_registered', PointCloud2, vision.pclCallback, queue_size = 1)
    
    ros.spin()
