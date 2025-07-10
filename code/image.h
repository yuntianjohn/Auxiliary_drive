//
// Created by c on 25-7-2.
//

#ifndef IMAGE_H
#define IMAGE_H
#include <opencv2/opencv.hpp>

//二值化
struct ProcessResult {
    cv::Mat binaryROI;                      // 二值化后的ROI图像
    cv::Mat colorROI;                       // 原始彩色ROI图像
};

ProcessResult processFrameOptimized(const cv::Mat& frame, const cv::Rect& roiRect);


cv::Mat inversePerspectiveTransform(const cv::Mat& src, const cv::Mat& inverseMatrix, int result_rows, int result_cols);

std::pair<int, int> find_special_region(const std::vector<int>& data);

int find_special_region_pro(const std::vector<int>& data);
#endif //IMAGE_H
