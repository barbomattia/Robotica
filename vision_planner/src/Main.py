#!/usr/bin/env python
# vision_node.py

import rospy
from vision_planner.srv import objectDetection,objectDetectionResponse

def image_callback(req):
    rospy.loginfo("request received")

    #xBlocksProva = [0.154898, 0.555929, 0.877663, 0.30047, 0.725948, 0.89356, 0.569056, 0.5285, 0.894334, 0.596488, 0.395709, 0.892513, 
    #                0.48072, 0.65344, 0.892218, 0.0809736, 0.433872, 0.88351, 0.350658, 0.434405, 0.896005, 0.303582, 0.599009, 0.895858,
    #                0.204657, 0.426884, 0.896583, 0.138785, 0.704752, 0.896002, 0.383786, 0.312797, 0.903243 ]
    #phiBlocksProva = [0, 0, -75.821, 0, 0, 169.966, 0, 0, -99.7265, 0, 0, -92.906, 0, 0, 35.0827, 0, 0, 19.5669, 0, 0, -144.856, 0, 0, -22.6999,
    #                  0, 0, 91.393, 0, 0, -13.1229, 0, 0, -75.821 ]
    #nameBlocksProva = ["X1-Y4-Z1", "X1-Y3-Z2", "X2-Y2-Z2-FILLET", "X1-Y4-Z2", "X1-Y2-Z2-TWINFILLET", "X1-Y2-Z1", "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2", "X1-Y3-Z2-FILLET", "X1-Y1-Z2", "X2-Y2-Z2"]

    xBlocksProva = [0.582973, 0.726230, 0.870204]
    phiBlocksProva = [0,0,2.72949]
    nameBlocksProva =["X1-Y4-Z2"]

    return objectDetectionResponse(xBlocks=xBlocksProva, phiBlocks=phiBlocksProva, nameBlocks=nameBlocksProva);


def vision_node():
    rospy.init_node('vision_node')
    s = rospy.Service('object_detection', objectDetection, image_callback)

    rospy.loginfo("vision_node ready")
    rospy.spin()



if __name__ == '__main__':
    vision_node()

