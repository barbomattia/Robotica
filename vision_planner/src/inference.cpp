#include "inference.hpp"

std::vector<std::string> classNames = {
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
};

cv::dnn::Net loadONNX(){
    cv::dnn::Net net = cv::dnn::readNetFromONNX(ONNX);

    if (cv::cuda::getCudaEnabledDeviceCount() > 0){
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    } else {
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    
    return net;
}

cv::Mat padAndResizeImage(const cv::Mat frame){
    cv::Mat resized = frame;
    int width  = frame.cols;
    int height = frame.rows;

    if (width != height){
        int max = width > height ? width : height;

        resized = cv::Mat::zeros(max, max, CV_8UC3);
        frame.copyTo(resized({0, 0, width, height}));
    }
    
    cv::resize(resized, resized, SIZE);

    return resized;
}


std::vector<Detection> Inference(const cv::Mat frame, cv::dnn::Net& net){
    cv::Mat resize = padAndResizeImage(frame);

    cv::Mat blob;
    cv::dnn::blobFromImage(resize, blob, 1.0/255.0, SIZE, cv::Scalar(), true, false);
    net.setInput(blob);
    
    std::vector<cv::Mat> results;
    net.forward(results, net.getUnconnectedOutLayersNames());

    cv::Mat output = cv::Mat(cv::Size(results[0].size[2], results[0].size[1]), CV_32F, (float *) results[0].data).t();
    int nWidth = output.cols;
    int rows  = output.rows;
    float* data = (float*) output.data;

    std::vector<int> ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> bboxes;
    std::vector<cv::Point> centroids;


    for (int i = 0; i < rows; ++i){
        cv::Mat scores(1, classNames.size(), CV_32FC1, data + 4);

        cv::Point id;
        double maxScore;
        cv::minMaxLoc(scores, 0, &maxScore, 0, &id);

        if (maxScore >= SCORETHRESH){
            //std::cout << "[" << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << "]\n";
            float centerX = data[0] * SCALEW;
            float centerY = data[1] * SCALEH;
            float width   = data[2] * SCALEW;
            float height  = data[3] * SCALEH;
            //std::cout << "[" << centerX << ", " << centerY << ", " << width << ", " << height << "]\n";

            int left = int(centerX - 0.5 * width);
            int top = int(centerY - 0.5 * height);

            ids.push_back(id.x);
            confidences.push_back(maxScore);
            bboxes.push_back(cv::Rect(left, top, width, height));
            centroids.push_back(cv::Point2i((int) centerX, (int) centerY));
        }

        data += nWidth;
    }

    std::vector<int> NMS;
    cv::dnn::NMSBoxes(bboxes, confidences, SCORETHRESH, NMSTHRESH, NMS);

    std::vector<Detection> detections;
    for (auto &&i : NMS){
        Detection detected;

        detected.classId = ids[i];
        detected.confidence = confidences[i];
        detected.bbox = bboxes[i];
        detected.centroid = centroids[i];
        detected.className = classNames[detected.classId];

        detections.push_back(detected);
    }

    return detections;
    
}

void showBboxes(cv::Mat& frame, const std::vector<Detection> detections, std::string window){
    for (auto &&detection : detections){
        cv::rectangle(frame, detection.bbox, cv::Scalar(0, 255, 0), 2);

        std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4);
        cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
        cv::Rect textBox(detection.bbox.x, detection.bbox.y - 40, textSize.width + 10, textSize.height + 20);

        cv::rectangle(frame, textBox, cv::Scalar(0, 255, 0), cv::FILLED);
        cv::putText(frame, classString, cv::Point(detection.bbox.x + 5, detection.bbox.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
    }

    cv::imshow("Detection", frame);
    cv::waitKey(-1);
}
