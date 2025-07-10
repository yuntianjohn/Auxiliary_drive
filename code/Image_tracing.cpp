#include "Image_tracing.h"

//计算面积
inline int64 calculateAreaSquared(const std::vector<cv::Point>& contour) {
    int64 area = 0;
    const int n = static_cast<int>(contour.size());

    // 处理退化情况：少于3个点的轮廓面积为0
    if (n < 3) return 0;

    // 直接访问底层数据以提高性能
    const cv::Point* pts = contour.data();

    // 使用格林公式计算有向面积的两倍
    // 公式：area = 0.5 * |Σ(x_i*y_{i+1} - x_{i+1}*y_i)|
    // 此处省略除以2以避免浮点运算，最终结果为面积的两倍
    for (int i = 0; i < n; ++i) {
        const cv::Point& p = pts[i];
        const cv::Point& q = pts[(i+1)%n];
        area += static_cast<int64>(p.x) * q.y - static_cast<int64>(q.x) * p.y;
    }

    // 返回绝对值以确保结果非负
    return std::abs(area);
}

void findLargestContour(const cv::Mat& binaryImage, std::vector<cv::Point>& largestContour) {
    // 清空输出容器
    largestContour.clear();

    // 定义存储轮廓和层次结构的容器
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // 执行轮廓检测（使用RETR_TREE模式保留完整层次结构）
    findContours(binaryImage, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    // 处理无轮廓的情况
    if (contours.empty()) {
        return;
    }

    // 初始化最大面积及其索引
    int64 maxAreaSq = 0;
    size_t maxAreaIndex = 0;

    // 遍历所有轮廓寻找最大面积
    for (size_t i = 0; i < contours.size(); ++i) {
        const auto& contour = contours[i];
        const int64 currentAreaSq = calculateAreaSquared(contour);
        if (currentAreaSq > maxAreaSq) {
            maxAreaSq = currentAreaSq;
            maxAreaIndex = i;
        }
    }

    // 输出最大轮廓
    largestContour = contours[maxAreaIndex];
}

void processLeftRightPoints(const std::vector<cv::Point>& points,
                           std::vector<cv::Point>& left_result,
                           std::vector<cv::Point>& right_result) {
    // 清空结果容器确保无残留数据
    left_result.clear();
    right_result.clear();

    // 处理空输入情况，避免无效计算
    if (points.empty()) {
        return;
    }

    // 确定Y坐标范围，用于初始化边界数组
    int max_y = 0;
    for (const auto& p : points) {
        if (p.y > max_y) max_y = p.y;
    }

    std::cout<<max_y<<std::endl;
    // 初始化左右边界数组，使用极值作为初始值便于后续比较
    std::vector<int> min_x(max_y + 1, INT_MAX);  // 存储每个Y对应的最小X（左边界）
    std::vector<int> max_x(max_y + 1, INT_MIN);  // 存储每个Y对应的最大X（右边界）

    // 遍历所有点，更新左右边界
    for (const auto& p : points) {
        // 更新当前Y坐标的左边界（最小X值）
        if (p.x < min_x[p.y]) {
            min_x[p.y] = p.x;
        }
        // 更新当前Y坐标的右边界（最大X值）
        if (p.x > max_x[p.y]) {
            max_x[p.y] = p.x;
        }
    }

    // 预分配内存以优化性能
    left_result.reserve(max_y + 1);
    right_result.reserve(max_y + 1);

    // 生成完整的左右边界点集
    for (int y = 0; y <= max_y; ++y) {
        // 左边界点：存在有效点则使用最小X，否则使用默认值(-1, y)
        left_result.emplace_back((min_x[y] != INT_MAX) ? cv::Point(min_x[y], y) : cv::Point(-1, y));
        // 右边界点：存在有效点则使用最大X，否则使用默认值(ROI_WIDTH, y)
        right_result.emplace_back((max_x[y] != INT_MIN) ? cv::Point(max_x[y], y) : cv::Point(160, y));
    }
}

bool isStraightLine(const std::vector<cv::Point>& points, double threshold) {
    // 边界检查：至少需要2个点才能构成直线
    if (points.size() < 2) {
        return false;
    }

    // 特殊情况：只有2个点时直接视为直线
    if (points.size() == 2) {
        return true;
    }

    // 1. 计算点集的均值
    double sumX = 0, sumY = 0;
    for (const auto& point : points) {
        sumX += point.x;
        sumY += point.y;
    }
    double meanX = sumX / points.size();
    double meanY = sumY / points.size();

    // 2. 计算最小二乘法直线参数 (y = ax + b)
    double sumXY = 0, sumX2 = 0;
    for (const auto& point : points) {
        sumXY += (point.x - meanX) * (point.y - meanY);
        sumX2 += (point.x - meanX) * (point.x - meanX);
    }

    // 防止除零错误
    if (sumX2 == 0) {
        return true; // 所有点x坐标相同，垂直直线
    }

    double a = sumXY / sumX2;    // 斜率
    double b = meanY - a * meanX; // 截距

    // 3. 计算所有点到直线的距离平方和
    double totalError = 0;
    for (const auto& point : points) {
        // 点到直线的距离公式: |ax - y + b| / sqrt(a² + 1)
        double distance = std::abs(a * point.x - point.y + b) / std::sqrt(a * a + 1);
        totalError += distance * distance;
    }

    // 4. 计算均方误差
    double meanError = std::sqrt(totalError / points.size());

    // 5. 判断是否小于阈值
    return meanError < threshold;
}


std::vector<int> calculateVerticalDistances(
    const std::vector<cv::Point>& points1,
    const std::vector<cv::Point>& points2,
    int imageHeight)
{
    // 创建结果数组，初始化为-1（无效值）
    std::vector<int> distances(imageHeight, -1);

    // 创建两个数组存储每个y坐标的有效x值
    std::vector<int> xValues1(imageHeight, -1);  // 点集1的x值，初始化为无效标记-1
    std::vector<int> xValues2(imageHeight, 160); // 点集2的x值，初始化为无效标记160

    // 填充点集1的x值
    for (const auto& pt : points1) {
        if (pt.y >= 0 && pt.y < imageHeight) {
            xValues1[pt.y] = pt.x;
        }
    }

    // 填充点集2的x值
    for (const auto& pt : points2) {
        if (pt.y >= 0 && pt.y < imageHeight) {
            xValues2[pt.y] = pt.x;
        }
    }

    // 计算每个y坐标的像素距离
    for (int y = 0; y < imageHeight; y++) {
        // 跳过无效点（点集1为-1或点集2为160）
        if (xValues1[y] == -1 || xValues2[y] == 160) {
            continue;
        }

        // 计算两点集在相同y坐标的像素距离
        distances[y] = std::abs(xValues2[y] - xValues1[y]);
    }

    return distances;
}

std::pair<int, int> findAbnormalRange(const std::vector<int>& data, int threshold) {
    int first = -1;
    int last = -1;

    // 查找第一个异常值
    for (size_t i = 0; i < data.size(); ++i) {
        if (data[i] > threshold) {
            first = i;
            break;
        }
    }

    // 如果找到第一个异常值，则查找最后一个异常值
    if (first != -1) {
        // 从尾部向前查找
        for (int i = data.size() - 1; i >= first; --i) {
            if (data[i] > threshold) {
                last = i;
                break;
            }
        }
    }

    return std::make_pair(first, last);
}


std::vector<cv::Point> getAllPointsOnLine(const cv::Point& pt1, const cv::Point& pt2, int numPoints) {
    // 检查输入的点数是否有效
    if (numPoints < 2) {
        return std::vector<cv::Point>();
    }

    // 用于存储计算得到的点的向量
    std::vector<cv::Point> points;
    // 预先为向量分配足够的空间，避免在添加元素时多次重新分配内存
    points.reserve(numPoints);
    // 计算两点在 x 轴方向上的差值
    int dx = pt2.x - pt1.x;
    // 计算两点在 y 轴方向上的差值
    int dy = pt2.y - pt1.y;

    // 循环 numPoints 次，计算线段上的每个点
    for (int i = 0; i < numPoints; ++i) {
        // 计算当前点在线段上的相对位置，t 的范围从 0 到 1
        float t = static_cast<float>(i) / (numPoints - 1);
        // 根据线性插值公式计算当前点的 x 坐标，加上 0.5f 是为了实现四舍五入
        int x = pt1.x + static_cast<int>(dx * t + 0.5f);
        // 根据线性插值公式计算当前点的 y 坐标，加上 0.5f 是为了实现四舍五入
        int y = pt1.y + static_cast<int>(dy * t + 0.5f);
        // 将计算得到的点添加到向量中
        points.emplace_back(x, y);
    }
    // 使用 std::move 避免不必要的拷贝，提高性能
    return std::move(points);
}

void replaceLongWithShort(std::vector<cv::Point>& long_vec, const std::vector<cv::Point>& short_vec) {
    // 遍历短向量中的每个点
    for (const auto& pt : short_vec) {
        // 检查当前点的 y 坐标是否在长向量的有效索引范围内
        if (pt.y >= 0 && pt.y < static_cast<int>(long_vec.size())) {
            // 如果在有效范围内，用短向量中的当前点替换长向量中相同 y 坐标位置的点
            long_vec[pt.y] = pt;
        }
    }
}

void find_center(std::vector<cv::Point>& left_points,
                 std::vector<cv::Point>& right_points,
                 std::vector<cv::Point>& center_points)
{
    // 预分配内存并获取裸指针
    constexpr int height = ROI_HEIGHT;
    center_points.resize(height);

    cv::Point* left_ptr = left_points.data();
    cv::Point* right_ptr = right_points.data();
    cv::Point* center_ptr = center_points.data();

    // 顺序访问优化缓存
    for(int y = 0; y < height; ++y)  // 改为正序遍历
    {
        const int lx = left_ptr[y].x;
        const int rx = right_ptr[y].x;
        center_ptr[y] = cv::Point((lx + rx) >> 1, y);  // 用位移代替除法
    }
}



