#ifndef INFERENCE_H
#define INFERENCE_H

#define ONNX "./yolov8.onnx"
#define SQUARE 640
#define SIZE cv::Size(SQUARE, SQUARE)
#define SCALEW 1024 / SQUARE
#define SCALEH 1024 / SQUARE
//#define SCALEW 1920 / SQUARE
//#define SCALEH 1080 / SQUARE
#define SCORETHRESH 0.35
#define NMSTHRESH 0.50

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

/**
 * @brief Main structure encapsulating all useful information about a single deteceted entity
 * 
 * This structure encapsulates information about a detected object, including
 * the class ID, confidence score, bounding box coordinates, centroid location,
 * and the class name.
*/
struct Detection{
    int classId{0};          /**< Class ID of the detected object. */
    float confidence {};     /**< Confidence score of the detection. */
    cv::Rect bbox {};        /**< Bounding box coordinates of the detected object. */
    cv::Point centroid {};   /**< Centroid location of the detected object. */
    std::string className{}; /**< Name of the class for the detected object. */
};

/**
 * @brief Load a neural network from an ONNX file.
 * 
 * This function loads a neural network model from an ONNX trained model file and configures it for inference.
 * If CUDA-enabled GPUs are available, and CUDA runtime is built on the system,
 * the function sets the backend and target to CUDA for faster inference.
 * Otherwise, it defaults to CPU processing.
 * 
 * @return A cv::dnn::Net object representing the laoded neural network.
 * 
 * @note This is a hard-coded setup, hence the 'yolov8.onnx' model is expected in the predefined folder
 * 
 * Example usage:
 * @code
 * cv::dnn::Net nn = loadONNX();
 * nn.setInput(blob);
 * nn.forward(results, nn.getUnconnectedLayersNames());
 * @endcode
 * 
 * @see cv::dnn::readNetFromONNX
 * @see cv::cuda::getCudaEnabledDeviceCount
*/
cv::dnn::Net loadONNX();

/**
 * @brief Resize and pad an input image to a square shape for compatibility with YOLOv8 netork input.
 * 
 * This function takes an input image and resizes it to a square shape while maintaining the original
 * aspect ratio. If the input image is different from a square shape, the aspect ratio is maintained 
 * and the rest of the image filled with zeroes (black pixels). Whichever the case, the image gets scaled
 * to a specific square resolution.
 * 
 * @param frame Input cv::Mat to be resized and padded.
 * 
 * @return Processed image.
 * 
 * @note The scaled image size is hard-coded to cv::Size(640, 640), needs to be changed if the network input expects another size
 * 
 * Example usage:
 * @code
 * cv::Mat input = cv::imread("path/to/image.jpeg");
 * input = padAndResizeImage(input);
 * @endcode
*/
cv::Mat padAndResizeImage(const cv::Mat frame);

/**
 * @brief Performs inference on an image based on a pre-trained YOLOv8 neural network
 * 
 * This function takes as an input an image, pre-processes it to the required standards set
 * by the neural network and forward passes the blob to the DNN. Afterwards, it recevives the
 * outputs in YOLO format [batchSize, 84, 8400] and processes them to extract meaningful data
 * about the bboxes, class types and confidence scores. Non-Maximum Suppression is then immediately
 * applied to filter out possible overlapping or invalid bboxes, and a list of all successful detections
 * is passed back.
 * 
 * @param frame Input cv::Mat to perform inference on.
 * @param net Pre-trained loaded ONNX neural network for object detection.
 * @return A vector of @ref Detection objects.
 * 
 * \note The format [batchSize, 84, 8400] is interpreted as:
 * 1 - batchSize (should always be 1 since the inference is made on singular frames)
 * 84 - 0, 1, 2, 3 are the usual YOLO bbox [centerX, centerY, width, height], the rest are the probabilities for each class
 * 8400 - number of possible detected objects
 * 
 * @note The classes are hard-coded, they can be replaced to work with a specific ONNX model.
 * 
 * Example usage:
 * @code
 * cv::dnn::Net nn = loadONNX();
 * cv::Mat input = cv::imread("path/to/image.jpeg");
 * std::vector<Detection> detections = Inference(input, nn);
 * @endcode
 * 
 * @see @ref padAndResizeImage()
 * @see @ref loadONNX() 
 * @see [Github issue](https://github.com/ultralytics/ultralytics/issues/2670) about YOLOv8 output
*/
std::vector<Detection> Inference(const cv::Mat frame, cv::dnn::Net& net);

void showBboxes(cv::Mat& frame, const std::vector<Detection> detections, std::string window);

extern std::vector<std::string> classNames;

#endif
