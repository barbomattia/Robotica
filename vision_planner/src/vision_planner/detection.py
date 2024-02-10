import cv2 as cv
import numpy as np
import os
import rospkg

## Path to ONNX model
ONNX        = 'yolov8.onnx'
## Size of network input
SQUARE      = 640
## XY size of network input
SIZE        = (SQUARE, SQUARE)
## Threshold for confidence scores
SCORETHRESH = 0.25
## Non-Maximum Suppression threshold
NMSTHRESH   = 0.50

## Classes of possibly detected objects
classNames = [
    "X1-Y1-Z2",
    "X1-Y2-Z1",
    "X1-Y2-Z2",
    "X1-Y2-Z2-CHAMFER",
    "X1-Y2-Z2-TWINFILLET",
    "X1-Y3-Z2",
    "X1-Y3-Z2-FILLET",
    "X1-Y4-Z1",
    "X1-Y4-Z2",
    "X2-Y2-Z2",
    "X2-Y2-Z2-FILLET"
]

##
#  @brief Main structure encapsulating all useful information about a single deteceted entity
#
#  This structure encapsulates information about a detected object, including
#  the class ID, confidence score, bounding box coordinates, centroid location,
#  and the class name.
#
#  More details.
class Detection:
    ## Only constructor, contains all the necessary parameters to represent the detection
    def __init__(self, _classId, _confidence, _bbox, _centroid, _className) -> None:
        self.classId    = _classId
        self.confidence = _confidence
        self.bbox       = _bbox
        self.centroid   = _centroid
        self.className  = _className

##
#  @brief Structure to get input image for neural network
#
#  This structure contains the cv::Mat object of the scaled and padded picture
#  for coherent inputing in the neural network, and the scaler to transform the
#  detected bboxes back to their original size on the image plane.
class Image:
    def __init__(self, _img: cv.Mat, _scaler: float) -> None:
        self.img    = _img
        self.scaler = _scaler

##
#  @brief Load a neural network from an ONNX file.
#  
#  This function loads a neural network model from an ONNX trained model file and configures it for inference.
#  If CUDA-enabled GPUs are available, and CUDA runtime is built on the system,
#  the function sets the backend and target to CUDA for faster inference.
#  Otherwise, it defaults to CPU processing.
#  
#  @return A cv2.dnn.Net object representing the loaded neural network.
#  
#  @note This is a hard-coded setup, hence the 'yolov8.onnx' model is expected in the predefined folder
#  
#  Example usage:
#  @code
#  nn = loadONNX()
#  nn.setInput(blob)
#  nn.forward(results, nn.getUnconnectedLayersNames())
#  @endcode
#  
#  @see cv2.dnn.readNetFromONNX
#  @see cv2.cuda.getCudaEnabledDeviceCount
def loadONNX() -> cv.dnn.Net:
    vision_path = rospkg.RosPack().get_path('vision_planner')
    net = cv.dnn.readNetFromONNX(os.path.join(vision_path, ONNX))
    
    if cv.cuda.getCudaEnabledDeviceCount() > 0:
        net.setPreferableBackend(cv.dnn.DNN_BACKEND_CUDA)
        net.setPreferableTarget(cv.dnn.DNN_TARGET_CUDA)
    else:
        net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)
    
    return net

##
#  @brief Resize and pad an input image to a square shape for compatibility with YOLOv8 network input.
#  
#  This function takes an input image and resizes it to a square shape while maintaining the original
#  aspect ratio. If the input image is different from a square shape, the aspect ratio is maintained 
#  and the rest of the image filled with zeroes (black pixels). Whichever the case, the image gets scaled
#  to a specific square resolution.
#  
#  @param frame Input cv2 image to be resized and padded.
#  
#  @return Processed image.
#  
#  @note The scaled image size is hard-coded to cv2.Size(640, 640), needs to be changed if the network input expects another size
#  
#  Example usage:
#  @code
#  input_image = cv2.imread("path/to/image.jpeg")
#  input_image = padAndResize(input_image)
#  @endcode
def padAndResize(frame: cv.Mat) -> Image:
    resize = frame.copy()
    width, height = frame.shape[1], frame.shape[0]
    dim = max(width, height)
    
    if width != height:
        resize = np.zeros(
            (dim, dim, 3), 
            dtype = np.uint8
        )
        resize[0:height, 0:width, :] = frame
        
    resize[:220, :] = [0, 0, 0]
    resize = cv.resize(resize, SIZE)
    resize = cv.cvtColor(resize, cv.COLOR_BGR2GRAY)
    resize = cv.cvtColor(resize, cv.COLOR_GRAY2BGR)
    image = Image(resize, dim / SQUARE)
    
    return image
    
##
#  @brief Performs inference on an image based on a pre-trained YOLOv8 neural network converted to ONNX.
#  
#  This function takes as input an image, pre-processes it to the required standards set
#  by the neural network, and forward passes the blob to the DNN. Afterwards, it receives the
#  outputs in YOLOv8 format [batchSize, 84, 8400] and processes them to extract meaningful data
#  about the bounding boxes, class types, and confidence scores. Non-Maximum Suppression is then immediately
#  applied to filter out possible overlapping or invalid bounding boxes, and a list of all successful detections
#  is passed back.
#  
#  @param frame Input cv2 image to perform inference on.
#  @param net Pre-trained loaded ONNX neural network for object detection.
#  
#  @return A list of Detection objects.
#  
#  \note The format [batchSize, 84, 8400] is interpreted as:
#  1 - batchSize (should always be 1 since the inference is made on singular frames)
#  84 - 0, 1, 2, 3 are the usual YOLO bounding box [centerX, centerY, width, height], the rest are the probabilities for each class
#  8400 - number of possible detected objects
#  
#  @note The classes are hard-coded; they can be replaced to work with a specific ONNX model.
#  
#  Example usage:
#  @code
#  nn = loadONNX()
#  input_image = cv2.imread("path/to/image.jpeg")
#  detections = inference(input_image, nn)
#  @endcode
#  
#  @see @ref padAndResize
#  @see @ref loadONNX
#  @see [Github issue](https://github.com/ultralytics/ultralytics/issues/2670) about YOLOv8 output
def inference(frame: cv.Mat, net: cv.dnn.Net) -> list:
    image = padAndResize(frame)
    resized = image.img
    scale = image.scaler
    
    blob = cv.dnn.blobFromImage(resized, 1.0/255.0, SIZE, (0, 0, 0), True, False)
    net.setInput(blob)
    
    results = net.forward(net.getUnconnectedOutLayersNames())
    results = results[0].transpose((0, 2, 1))

    rows = results[0].shape[0]
    
    ids = []
    scores = []
    bboxes = []
    centroids = []
    
    for i in range(rows):
        data = results[0][i]
        score = data[4]
        
        classScores = data[4:]
        _, _, _, maxScore = cv.minMaxLoc(classScores)
        classId = maxScore[1]
        if classScores[classId] > SCORETHRESH:
            centerX = data[0].item() * scale
            centerY = data[1].item() * scale
            
            width  = data[2].item() * scale
            height = data[3].item() * scale
            
            left = int(centerX - 0.5 * width)
            top  = int(centerY - 0.5 * height)
            
            ids.append(classId)
            scores.append(classScores[classId])
            bboxes.append((left, top, int(width), int(height)))
            centroids.append((int(centerX), int(centerY)))
        
        indexes = cv.dnn.NMSBoxes(bboxes, scores, SCORETHRESH, NMSTHRESH)
        
        detections = []
        for idx in indexes:
            detected = Detection(ids[idx], scores[idx], bboxes[idx], centroids[idx], classNames[ids[idx]])
            detections.append(detected)
            
    return detections

##
#  @brief Show on screen the image on which inference has been performed with the detections 
#
#  The function takes as input the image used for detection and the subsequent detections
#  that the model has provided. Following this, bounding boxes are drawn on the image
#  with the respective class names and confidence scores and shown on screen for any
#  necessary assesment.
#
#  @param frame Input cv2 image on which inference has been performed.
#  @param detections list of Detection objects with data for all the detected blocks.  
#
#  Example usage:
#  @code
#  nn = loadONNX()
#  input_image = cv2.imread("path/to/image.jpeg")
#  detections = inference(input_image, nn)
#  showBBox(input_image, detections)
#  @endcode
#
#  @see @ref padAndResize
#  @see @ref loadONNX
#  @see @ref inference
def showBBox(frame: cv.Mat, detections: list) -> None:
    for detection in detections:
        cv.rectangle(
            img = frame,
            pt1 = (detection.bbox[0], detection.bbox[1]),
            pt2 = (detection.bbox[0] + detection.bbox[2], detection.bbox[1] + detection.bbox[3]),
            color = (0, 255, 0),
            thickness = 2
        )
        
    for detection in detections:
        
        string = f'{detection.className} {detection.confidence:.2f}'
        textSize = cv.getTextSize(string, cv.FONT_HERSHEY_DUPLEX, 0.5, 1)[0]
        textBox = (detection.bbox[0] - 1, detection.bbox[1], detection.bbox[0] + textSize[0] + 10, detection.bbox[1] - textSize[1] - 10)
        
        cv.rectangle(
            frame,
            (textBox[0], textBox[1]),
            (textBox[2], textBox[3]),
            (0, 255, 0),
            cv.FILLED
        )
        
        cv.putText(
            img = frame,
            text = string,
            org = (detection.bbox[0] + 5, detection.bbox[1] - 5),
            fontFace = cv.FONT_HERSHEY_DUPLEX,
            fontScale = 0.5,
            color = (0, 0, 0),
            thickness = 1,
            lineType = 0
        )
    
    x = frame.shape[0]
    y = frame.shape[1]
    ratio = x / y
    if x > y:
        x = 600
        y = x * ratio
    else:
        x = 600 * 1 / ratio
        y = 600
    
    frame = cv.resize(frame, (int(x), int(y)))
    cv.imshow('Window', frame)
    cv.waitKey(0)
    cv.destroyAllWindows()
        
if __name__ == '__main__':
    net = loadONNX()
    frame = cv.imread(f'./photos/photo1.jpg')
    detections = inference(frame, net)
    showBBox(frame, detections)
    """for i in range(13):
        frame = cv.imread(f'./photos/photo{i}.jpg') 
        detections = inference(frame, net)
        showBBox(frame, detections)"""
