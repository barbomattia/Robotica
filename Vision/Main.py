# vision_node.py
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Import the String message type
from your_yolo_module import detect_lego_blocks  # Import your YOLO module




def image_callback(msg):
    # Process the image using YOLO
    detected_blocks = detect_lego_blocks(msg)

    # Publish the detected block positions as a string
    block_positions_str = ",".join(map(str, detected_blocks))
    block_positions_pub.publish(block_positions_str)

def vision_node():
    rospy.init_node('vision_node', anonymous=True)
    rospy.Subscriber('/your/3d_camera/image', Image, image_callback)

    # Add a publisher for block positions
    global block_positions_pub
    block_positions_pub = rospy.Publisher('/vision_node/detected_blocks', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    vision_node()

