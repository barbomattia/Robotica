#!/usr/bin/env python
# vision_node.py

import rospy
from vision_planner.srv import objectDetection,objectDetectionResponse

def image_callback(req):
    rospy.loginfo("request received")

    xBlocksProva = [0.615, 0.480, 0.87,
                    0.590, 0.716, 0.896397,     0.229, 0.262, 0.894732,    
                    0.169, 0.629, 0.896958,     0.369, 0.273, 0.893231,  
                    0.449, 0.582, 0.877484,     0.255, 0.462, 0.904036,
                    0.606, 0.345, 0.86939,      0.125, 0.415, 0.893645,
                    0.296, 0.729, 0.89282,      0.433, 0.720, 0.892169,  ]
    phiBlocksProva = [0, 0, -1.20, 
                      0, 0, -0.37,
                      0, 0, 0.484, 
                      0, 0, 0.9719, 
                      0, 0, 0.7357, 
                      0, 0, 2.7697, 
                      0, 0, 0.682,
                      0, 0, -2.49,
                      0, 0, 1.402, 
                      0, 0, 1.406, 
                      0, 0, 2.713 ]
    nameBlocksProva = [ 
                       "X1-Y3-Z2", 
                       "X1-Y2-Z2-CHAMFER", 
                       "X1-Y3-Z2-FILLET",
                       "X1-Y1-Z2",
                       "X1-Y4-Z2", 
                       "X2-Y2-Z2",
                       "X1-Y2-Z1",
                       "X1-Y2-Z2-TWINFILLET",
                       "X2-Y2-Z2-FILLET",
                       "X1-Y4-Z1",
                       "X1-Y2-Z2"]

   

    return objectDetectionResponse(xBlocks=xBlocksProva, phiBlocks=phiBlocksProva, nameBlocks=nameBlocksProva);


def vision_node():
    rospy.init_node('vision_node')
    s = rospy.Service('object_detection', objectDetection, image_callback)

    rospy.loginfo("vision_node ready")
    rospy.spin()



if __name__ == '__main__':
    vision_node()

 