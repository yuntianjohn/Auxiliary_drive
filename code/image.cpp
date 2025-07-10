#include "image.h"

ProcessResult processFrameOptimized(const cv::Mat& frame, const cv::Rect& roiRect) {
    // 定义结果结构体实例
    ProcessResult result;

    // 提取ROI区域（浅拷贝，共享数据缓冲区）
    result.colorROI = frame(roiRect);

    // 灰度转换与二值化处理流程
    // Step 1: 创建临时灰度图像存储中间结果
    cv::Mat grayROI;
    // Step 2: 将彩色ROI转换为灰度图像（CV_8UC1格式）
    cv::cvtColor(result.colorROI, grayROI, cv::COLOR_BGR2GRAY);
    // Step 3: 应用Otsu自动阈值二值化算法
    cv::threshold(grayROI, result.binaryROI, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 返回处理结果
    return result;
}

cv::Mat map_x, map_y;
bool map_initialized = false;
cv::Mat inversePerspectiveTransform(const cv::Mat& src, const cv::Mat& inverseMatrix, int result_rows, int result_cols){
    CV_Assert(src.channels() == 1);
    CV_Assert(inverseMatrix.rows == 3 && inverseMatrix.cols == 3 && inverseMatrix.type() == CV_64F);

    if (!map_initialized) {
        // 构造目标图像的齐次坐标矩阵
        cv::Mat dst_homo(3, result_rows * result_cols, CV_64F);
        for (int y = 0; y < result_rows; ++y) {
            for (int x = 0; x < result_cols; ++x) {
                int idx = y * result_cols + x;
                dst_homo.at<double>(0, idx) = x;
                dst_homo.at<double>(1, idx) = y;
                dst_homo.at<double>(2, idx) = 1.0;
            }
        }

        // 逆透视映射
        cv::Mat mapped = inverseMatrix * dst_homo;

        // 归一化
        for (int i = 0; i < mapped.cols; ++i) {
            double w = mapped.at<double>(2, i);
            if (w != 0.0) {
                mapped.at<double>(0, i) /= w;
                mapped.at<double>(1, i) /= w;
            }
        }

        // 初始化映射图 map_x 和 map_y
        map_x.create(result_rows, result_cols, CV_32F);
        map_y.create(result_rows, result_cols, CV_32F);
        for (int i = 0; i < result_rows; ++i) {
            for (int j = 0; j < result_cols; ++j) {
                int idx = i * result_cols + j;
                map_x.at<float>(i, j) = static_cast<float>(mapped.at<double>(0, idx));
                map_y.at<float>(i, j) = static_cast<float>(mapped.at<double>(1, idx));
            }
        }

        map_initialized = true;
    }

    // 直接使用缓存的 map 进行 remap
    cv::Mat dst;
    cv::remap(src, dst, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    return dst;
}

std::pair<int, int> find_special_region(const std::vector<int>& data) {
    int n = data.size();
    // 数组太小，无法形成区域
    if (n < 2) {
        return {-1, -1};
    }

    int start_index = -1;
    int end_index = -1;
    bool found_end = false; // 是否找到结束索引

    // 从最后一个索引开始向前遍历（索引从 n-1 到 1）
    for (int i = n - 1; i >= 1; --i) {
        if (!found_end) {
            // 检测进入感兴趣区域的跳变：data[i-1] 明显大于 data[i]
            if (data[i - 1] > data[i] + 5) {
                end_index = i - 1; // 标记结束索引
                found_end = true;
            }
        } else {
            // 在感兴趣区域内，检测离开区域的跳变：data[i-1] 明显小于 data[i]
            if (data[i - 1] < data[i] - 5) {
                start_index = i; // 标记开始索引
                break; // 找到开始索引，提前结束遍历
            }
        }
    }

    // 已找到结束索引但未找到开始索引，说明区域扩展到数组头部
    if (found_end && start_index == -1) {
        start_index = 0;
    }

    // 返回结果
    if (found_end) {
        return {start_index, end_index};
    } else {
        return {-1, -1}; // 未找到区域
    }
}

//找远端车库点
int find_special_region_pro(const std::vector<int>& data) {
    int n = data.size();
    // 数组太小，无法形成区域
    if (n < 2) {
        return -1;
    }

    int start_index = -1;
    bool found_end = false; // 是否找到结束索引

    // 从最后一个索引开始向前遍历（索引从 n-1 到 1）
    for (int i = n - 1; i >= 1; --i) {
        if (!found_end) {
            // 检测进入感兴趣区域的跳变：data[i-1] 明显大于 data[i]
            if (data[i - 1] < data[i] - 5) {
                start_index = i;
                found_end = true;
            }
        }
    }

    // 已找到结束索引但未找到开始索引，说明区域扩展到数组头部
    if (found_end && start_index == -1) {
        start_index = 0;
    }

    // 返回结果
    if (found_end) {
        return start_index;
    } else {
        return -1; // 未找到区域
    }
}