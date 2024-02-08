#!/usr/bin/env python
# vision_node.py

import rospy
from vision_planner.srv import objectDetection,objectDetectionResponse

def image_callback(req):
    rospy.loginfo("request received")

    xBlocksProva = [0.59, 0.47, 1.1, 0.43, 0.55, 1.1, 0.54, 0.32, 1.1]
    phiBlocksProva = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    nameBlocksProva = ["X2-Y2-Z2", "X1-Y2-Z1", "X1-Y3-Z2"]

    return objectDetectionResponse(xBlocks=xBlocksProva, phiBlocks=phiBlocksProva, nameBlocks=nameBlocksProva);


def vision_node():
    rospy.init_node('vision_node')
    s = rospy.Service('object_detection', objectDetection, image_callback)

    rospy.loginfo("vision_node ready")
    rospy.spin()



if __name__ == '__main__':
    vision_node()