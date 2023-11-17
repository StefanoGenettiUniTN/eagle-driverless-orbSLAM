#ifndef INFERENCE_H
#define INFERENCE_H

// Cpp native
#include <fstream>
#include <vector>
#include <string>
#include <random>

// OpenCV / DNN / Inference
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

namespace YOLO
{
    struct Detection
    {
        int class_id{0};
        std::string className{};
        float confidence{0.0};
        cv::Scalar color{};
        cv::Rect box{};
        cv::Mat boxMask;
    };

    class Inference
    {
    public:
        Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape = {640, 640}, const std::string &classesTxtFile = "", const bool &runWithCuda = true);
        std::vector<Detection> runInference(const cv::Mat &input);

    private:
        void loadClassesFromFile();
        void loadOnnxNetwork();
        cv::Mat formatToSquare(const cv::Mat &source);

        cv::Mat GetMask(const cv::Mat &maskProposal, const cv::Mat &mask_protos, cv::Rect &temp_rect, cv::Size src_img_shape);

        std::string modelPath{};
        std::string classesPath{};
        bool cudaEnabled{};

        std::vector<std::string> classes{"seg_blue_cone", "seg_large_orange_cone", "seg_orange_cone", "seg_unknown_cone", "seg_yellow_cone"};
        std::vector<cv::Scalar> colors{cv::Scalar(190, 100, 20), cv::Scalar(0, 110, 255), cv::Scalar(0, 110, 255), cv::Scalar(127, 127, 127), cv::Scalar(60, 255, 255)};

        cv::Size2f modelShape{};


        int seg_channels = 32;
        float mask_threshold = 0.5;
        int seg_width = 320;
        int seg_height = 192;
        int net_width = 1280;
        int net_height = 768;

        float modelConfidenceThreshold {0.25};
        float modelScoreThreshold      {0.45};
        float modelNMSThreshold        {0.50};

        bool letterBoxForSquare = true;

        cv::dnn::Net net;
    };
}

#endif // INFERENCE_H
