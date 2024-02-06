#!/usr/bin/env python
# vision_node.py

import rospy
from vision_planner.srv import objectDetection,objectDetectionResponse

def image_callback(req):
    rospy.loginfo("request received")

    xBlocksProva = [0.9, 0.25, 1.1, 1.0, 1.0, 0.5, 1.0, 1.0, 0.5]
    phiBlocksProva = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    nameBlocksProva = ["X1-Y4-Z2", "bloc2", "bloc3"]

    return objectDetectionResponse(xBlocks=xBlocksProva, phiBlocks=phiBlocksProva, nameBlocks=nameBlocksProva);


def vision_node():
    rospy.init_node('vision_node')
    s = rospy.Service('object_detection', objectDetection, image_callback)

    rospy.loginfo("vision_node ready")
    rospy.spin()



if __name__ == '__main__':
    vision_node()