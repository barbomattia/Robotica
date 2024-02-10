#!/usr/bin/env python
# vision_node.py

import rospy
from vision_planner.srv import objectDetection,objectDetectionResponse

def image_callback(req):
    rospy.loginfo("request received")

    xBlocksProva = [0.245727, 0.475905, 0.896004, 0.394039, 0.275355, 0.892606, 0.598226, 0.360282, 0.893955, 0.192153, 0.322698, 0.893202, 
                    0.420771, 0.693625, 0.902175, 0.575869, 0.744789, 0.886947, 0.287196, 0.688195, 0.891654, 0.100981, 0.495092, 0.904545,
                    0.0471689, 0.302103, 0.892379, 0.534214, 0.515961, 0.899191 ]
    phiBlocksProva = [0, 0, 52.5198, 0, 0, 41.7293, 0, 0, -2.98438, 0, 0, 55.4589, 0, 0, 7.52428, 0, 0, 89.2944, 0, 0, 1.19949, 0, 0, -30.4401,
                      0, 0, 92.4665, 0, 0, -18.6404 ]
    nameBlocksProva = ["X1-Y2-Z1", "X1-Y4-Z2", "X1-Y2-Z2-CHAMFER", "X1-Y2-Z2", "X2-Y2-Z2-FILLET", "X1-Y2-Z2-TWINFILLET", "X1-Y3-Z2-FILLET", "X2-Y2-Z2", "X1-Y1-Z2", "X1-Y3-Z2"]

    return objectDetectionResponse(xBlocks=xBlocksProva, phiBlocks=phiBlocksProva, nameBlocks=nameBlocksProva);


def vision_node():
    rospy.init_node('vision_node')
    s = rospy.Service('object_detection', objectDetection, image_callback)

    rospy.loginfo("vision_node ready")
    rospy.spin()



if __name__ == '__main__':
    vision_node()


#  Blocco: name X1-Y4-Z1, x [0.0685232, 0.64095, 0.896014], phi [0, 0, 37.9417]
#  Blocco: name X1-Y4-Z2, x [0.394039, 0.275355, 0.892606], phi [0, 0, 41.7293]
#  Blocco: name X1-Y2-Z2-CHAMFER, x [0.598226, 0.360282, 0.893955], phi [0, 0, -2.98438]
#  Blocco: name X1-Y2-Z2, x [0.192153, 0.322698, 0.893202], phi [0, 0, 55.4589]
#  Blocco: name X2-Y2-Z2-FILLET, x [0.420771, 0.693625, 0.902175], phi [0, 0, 7.52428]
#  Blocco: name X1-Y2-Z2-TWINFILLET, x [0.575869, 0.744789, 0.886947], phi [0, 0, 89.2944]
#  Blocco: name X1-Y3-Z2-FILLET, x [0.287196, 0.688195, 0.891654], phi [0, 0, 1.19949]
#  Blocco: name X2-Y2-Z2, x [0.100981, 0.495092, 0.904545], phi [0, 0, -30.4401]
#  Blocco: name X1-Y1-Z2, x [0.0471689, 0.302103, 0.892379], phi [0, 0, 92.4665]
#  Blocco: name X1-Y3-Z2, x [0.534214, 0.515961, 0.899191], phi [0, 0, -18.6404]