#include "inference.h"

namespace YOLO
{
    Inference::Inference(const std::string &onnxModelPath, const cv::Size &modelInputShape, const std::string &classesTxtFile, const bool &runWithCuda)
    {
        modelPath = onnxModelPath;
        modelShape = modelInputShape;
        classesPath = classesTxtFile;
        cudaEnabled = runWithCuda;

        net_width = modelShape.width;
        net_height = modelShape.height;
        seg_width = net_width/4;
        seg_height = net_height/4;

        loadOnnxNetwork();
        // loadClassesFromFile(); The classes are hard-coded for this example
    }

    std::vector<Detection> Inference::runInference(const cv::Mat &input)
    {
        cv::Mat modelInput = input;
        if (letterBoxForSquare && modelShape.width == modelShape.height)
            modelInput = formatToSquare(modelInput);

        cv::Mat blob;
        cv::dnn::blobFromImage(modelInput, blob, 1.0/255.0, modelShape, cv::Scalar(), true, false);
        net.setInput(blob);

        std::vector<cv::Mat> outputs;
        net.forward(outputs, net.getUnconnectedOutLayersNames());

        int rows = outputs[0].size[1];
        int dimensions = outputs[0].size[2];
        //std::cout << outputs[0].size << std::endl;
        std::vector<int> mask_protos_shape = {(int)outputs[1].size[0],(int)outputs[1].size[1],(int)outputs[1].size[2],(int)outputs[1].size[3] };
        int mask_protos_length = outputs[1].size[0]*outputs[1].size[1]*outputs[1].size[2]*outputs[1].size[3];

        bool yolov8 = false;
        // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
        // yolov8 has an output of shape (batchSize, 5+4+32,  8400) (Num classes + box[x,y,w,h])
        if (dimensions > rows) // Check if the shape[2] is more than shape[1] (yolov8)
        {
            yolov8 = true;
            rows = outputs[0].size[2];
            dimensions = outputs[0].size[1];

            outputs[0] = outputs[0].reshape(1, dimensions);
            cv::transpose(outputs[0], outputs[0]);
        }
        //std::cout << outputs[0].size << std::endl;
        //std::cout << outputs[1].size << std::endl;
        float *data = (float *)outputs[0].data;

        float x_factor = modelInput.cols / modelShape.width;
        float y_factor = modelInput.rows / modelShape.height;

        //std::cout << modelInput.cols << " " << modelShape.width << " " << modelInput.rows << " " << modelShape.height << " " << std::endl;
        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        std::vector<std::vector<float>> picked_proposals;

        for (int i = 0; i < rows; ++i)
        {
            if (yolov8)
            {
                float *classes_scores = data+4;
                std::vector<float> mask_point(data + 4 + classes.size(), data + dimensions);
            

                cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
                cv::Point class_id;
                double maxClassScore;

                minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

                if (maxClassScore > modelScoreThreshold)
                {
                    confidences.push_back(maxClassScore);
                    class_ids.push_back(class_id.x);
                    picked_proposals.push_back(mask_point);
                    //net_width == dimensions
                    //segChannels == number of elements of sementation in net_width == 16
                    //net_height == rows
                    
                    //cv::Mat data2(1, dimensions, CV_32FC1, data);
                    //std::cout << data2 << std::endl;

                    float x = data[0];
                    float y = data[1];
                    float w = data[2];
                    float h = data[3];

                    //std::cout << "size: " << x << " " << y << " " << w << " "  << h << std::endl;

                    int left = MAX(int(x - 0.5 * w) * x_factor, 0);
                    int top = MAX(int(y - 0.5 * h) * y_factor, 0);

                    int width = MIN(int(w * x_factor), modelInput.cols - left);
                    int height = MIN(int(h * y_factor), modelInput.rows - top);
                    //std::cout << "size rect: " << left << " " << top << " " << width << " "  << height << std::endl;

                    boxes.push_back(cv::Rect(left, top, width, height));
                }
            }
            else // yolov5
            {
                float confidence = data[4];

                if (confidence >= modelConfidenceThreshold)
                {
                    float *classes_scores = data+5;
                    cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
                    cv::Point class_id;
                    double max_class_score;

                    minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

                    if (max_class_score > modelScoreThreshold)
                    {
                        confidences.push_back(confidence);
                        class_ids.push_back(class_id.x);

                        float x = data[0];
                        float y = data[1];
                        float w = data[2];
                        float h = data[3];

                        int left = int((x - 0.5 * w) * x_factor);
                        int top = int((y - 0.5 * h) * y_factor);

                        int width = int(w * x_factor);
                        int height = int(h * y_factor);

                        boxes.push_back(cv::Rect(left, top, width, height));
                    }
                }
            }

            data += dimensions;
        }
        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

        std::vector<Detection> detections{};
        //std::vector<std::vector<float>> temp_mask_proposals;
        for (unsigned long i = 0; i < nms_result.size(); ++i)
        {
            int idx = nms_result[i];

            Detection result;
            result.class_id = class_ids[idx];
            result.confidence = confidences[idx];
            //temp_mask_proposals.push_back(picked_proposals[idx]);
            
            /*
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(100, 255);
            result.color = cv::Scalar(dis(gen),
                                    dis(gen),
                                    dis(gen));
            */
            result.color = colors[result.class_id];

            result.className = classes[result.class_id];
            result.box = boxes[idx];
            //std::cout << boxes[idx].x << " " << boxes[idx].y << " " << boxes[idx].width << " " << boxes[idx].height << std::endl;

            result.boxMask = Inference::GetMask(cv::Mat(picked_proposals[idx]), outputs[1], result.box, input.size());
            detections.push_back(result);
        }
        return detections;
    }

    cv::Mat Inference::GetMask(const cv::Mat& maskProposal, const cv::Mat& mask_protos, cv::Rect& temp_rect, cv::Size src_img_shape) {
        //crop from mask_protos
        int rang_x = floor((temp_rect.x / float(net_width)) * seg_width);
        int rang_y = floor((temp_rect.y / float(net_height)) * seg_height);
        int rang_w = ceil((((temp_rect.x + temp_rect.width)) / float(net_width)) * seg_width) - rang_x;
        int rang_h = ceil((((temp_rect.y + temp_rect.height)) / float(net_height)) * seg_height) - rang_y;

        //std::cout << "rang:" << rang_x << " " << rang_y << " " << rang_w << " "  << rang_h << std::endl;
        
        rang_w = MAX(rang_w, 1);
        rang_h = MAX(rang_h, 1);
        if (rang_x + rang_w > seg_width) {
            if (seg_width - rang_x > 0)
                rang_w = seg_width - rang_x;
            else
                rang_x -= 1;
        }
        if (rang_y + rang_h > seg_height) {
            if (seg_height - rang_y > 0)
                rang_h = seg_height - rang_y;
            else
                rang_y -= 1;
        }

        //std::cout << "rang:" << rang_x << " " << rang_y << " " << rang_w << " "  << rang_h << std::endl;

        std::vector<cv::Range> roi_rangs;
        roi_rangs.push_back(cv::Range(0, 1));
        roi_rangs.push_back(cv::Range::all());
        roi_rangs.push_back(cv::Range(rang_y, rang_h + rang_y));
        roi_rangs.push_back(cv::Range(rang_x, rang_w + rang_x));
        //std::cout << roi_rangs << std::endl;
        

        //std::cout << "before crop" << std::endl;

        //crop
        //std::cout << "mask_protos shape: " << mask_protos.size << std::endl;
        cv::Mat temp_mask_protos = mask_protos(roi_rangs).clone();
        //std::cout << "out1" << std::endl;
        cv::Mat protos = temp_mask_protos.reshape(0, {seg_channels, rang_w * rang_h});
        //std::cout << "protos shape "<< protos.size << std::endl;
        //std::cout << "mask proposal shape " << maskProposal.size << std::endl;
        cv::Mat matmul_res = (maskProposal.t()*protos).t();
        //std::cout << "matmul_res shape " <<matmul_res.size<< std::endl;
        cv::Mat masks_feature = matmul_res.reshape(1, {rang_h, rang_w});
        //std::cout << "masks_feature shape " << masks_feature.size << std::endl;
        cv::Mat dest, mask;

        //sigmoid
        cv::exp(-masks_feature, dest);
        dest = 1.0 / (1.0 + dest);

        int left = floor(((net_width / float(seg_width)) * ((temp_rect.x / float(net_width)) * seg_width)));
        int top = floor(((net_height / float(seg_height)) * ((temp_rect.y / float(net_height)) * seg_height)));
        int width = ceil((net_width / float(seg_width)) * (((((temp_rect.x + temp_rect.width)) / float(net_width)) * seg_width) - rang_x));
        int height = ceil((net_height / float(seg_height)) * (((((temp_rect.y + temp_rect.height)) / float(net_height)) * seg_height) - rang_y));
        //std::cout << "mask:" << left << " " << top << " " << width << " "  << height << std::endl;

        cv::resize(dest, mask, cv::Size(width, height), cv::INTER_NEAREST);
        //std::cout << mask.size << std::endl;
        //std::cout << temp_rect << std::endl;
        //std::cout << temp_rect - cv::Point(left, top) << std::endl;
        mask = mask(temp_rect - cv::Point(left, top)) > 0.5;
        //std::cout << "before return" << std::endl;
        return mask;
    }

    void Inference::loadClassesFromFile()
    {
        std::ifstream inputFile(classesPath);
        if (inputFile.is_open())
        {
            std::string classLine;
            while (std::getline(inputFile, classLine))
                classes.push_back(classLine);
            inputFile.close();
        }
    }

    void Inference::loadOnnxNetwork()
    {
        std::cout << "\nTest" << std::endl;
        net = cv::dnn::readNetFromONNX(modelPath);
        std::cout << "\nTest2" << std::endl;
        if (cudaEnabled)
        {
            std::cout << "\nRunning on CUDA" << std::endl;
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        }
        else
        {
            std::cout << "\nRunning on CPU" << std::endl;
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        }
    }

    cv::Mat Inference::formatToSquare(const cv::Mat &source)
    {
        int col = source.cols;
        int row = source.rows;
        int _max = MAX(col, row);
        cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
        source.copyTo(result(cv::Rect(0, 0, col, row)));
        return result;
    }
}