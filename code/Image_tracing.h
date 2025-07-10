//
// Created by c on 25-7-2.
//

#ifndef IMAGE_TRACING_H
#define IMAGE_TRACING_H
#include "all.h"

//找最大轮廓
void findLargestContour(const cv::Mat& binaryImage, std::vector<cv::Point>& largestContour);

//寻找左右边线
void processLeftRightPoints(const std::vector<cv::Point>& points,
                           std::vector<cv::Point>& left_result,
                           std::vector<cv::Point>& right_result);

//直线判断
bool isStraightLine(const std::vector<cv::Point>& points, double threshold = 0.8);

//计算像素距离
std::vector<int> calculateVerticalDistances(
    const std::vector<cv::Point>& points1,
    const std::vector<cv::Point>& points2,
    int imageHeight = 120);

//根据像素距离判断断点
std::pair<int, int> findAbnormalRange(const std::vector<int>& data, int threshold = 65);

//根据两点划线
std::vector<cv::Point> getAllPointsOnLine(const cv::Point& pt1, const cv::Point& pt2, int numPoints);

//替换
void replaceLongWithShort(std::vector<cv::Point>& long_vec, const std::vector<cv::Point>& short_vec);


void find_center(std::vector<cv::Point>& left_points,
                 std::vector<cv::Point>& right_points,
                 std::vector<cv::Point>& center_points);
#endif //IMAGE_TRACING_H
