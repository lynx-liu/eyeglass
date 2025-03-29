///////////////////////////////////////////////////
//				date: 2025.01.10
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include "Contour.h"
#include <future>

// 计算两点之间的欧几里得距离
inline double distance(const cv::Point2f& pt1, const cv::Point2f& pt2) {
    return std::sqrt(std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2));
}

// 函数定义：将 std::vector<cv::Point> 转换为 std::vector<cv::Point2f>
std::vector<cv::Point2f> convertToPoint2f(const std::vector<cv::Point>& contour) {
    std::vector<cv::Point2f> result;
    result.reserve(contour.size() + 1); // 预分配空间以提高性能，可能需要多一个点

    for (const auto& point : contour) {
        result.emplace_back(point);
    }

    // 如果首尾点不同，手动封闭轮廓
    if (!contour.empty() && contour.front() != contour.back()) {
        result.emplace_back(contour.front());
    }
    return result;
}

std::vector<cv::Point> convertToPoint(const std::vector<cv::Point2f>& contour) {
    std::vector<cv::Point> result;
    result.reserve(contour.size() + 1); // 预分配空间以提高性能，可能需要多一个点

    for (const auto& point : contour) {
        result.emplace_back(point);
    }

    // 如果首尾点不同，手动封闭轮廓
    if (!contour.empty() && contour.front() != contour.back()) {
        result.emplace_back(contour.front());
    }
    return result;
}

// 使用点偏移直接缩放轮廓N个像素点，N<0为放大
std::vector<cv::Point2f> scaleContour(const std::vector<cv::Point2f>& contour, int N) {
    cv::Point2f center(0, 0);
    for (const auto& point : contour) {
        center += point;
    }
    center /= (int)contour.size();

    std::vector<cv::Point2f> scaleContour;
    scaleContour.reserve(contour.size());
    for (const auto& point : contour) {
        cv::Point2f direction = point - center;
        double length = distance(point, center);
        if (length > 0) direction /= length;

        scaleContour.push_back(point - direction * N);
    }
    return scaleContour;
}

int findMaxContourId(const std::vector<std::vector<cv::Point> >& contours)
{
    int index = 0, n = (int)contours.size();
    int maxArea = cv::minAreaRect(contours[index]).boundingRect().area();
    for (int i = 1; i < n; i++) {
        int area = cv::minAreaRect(contours[i]).boundingRect().area();
        if (area > maxArea) {
            index = i;
            maxArea = area;
        }
    }
    return index;
}

//查找拐点，建议先用贝赛尔曲线平滑后再查找
std::vector<cv::Point2f> findCornerPoints(const std::vector<cv::Point2f>& contour, double angleThreshold) {
    std::vector<cv::Point2f> cornerPoints;
    if (contour.size() < 3) return cornerPoints; // 至少需要3个点才能计算角度

    for (size_t i = 0; i < contour.size(); ++i) {
        cv::Point2f A = contour[(i - 1 + contour.size()) % contour.size()];
        cv::Point2f B = contour[i];
        cv::Point2f C = contour[(i + 1) % contour.size()];

        cv::Point2f AB = B - A;
        cv::Point2f BC = C - B;

        double dotProduct = (double)AB.x * BC.x + AB.y * BC.y;
        double magnitudeAB = std::sqrt(AB.x * AB.x + AB.y * AB.y);
        double magnitudeBC = std::sqrt(BC.x * BC.x + BC.y * BC.y);

        if (magnitudeAB > 1e-6 && magnitudeBC > 1e-6) { // 避免除零
            double cosTheta = dotProduct / (magnitudeAB * magnitudeBC);
            cosTheta = std::max(-1.0, std::min(1.0, cosTheta)); // 限制在 [-1,1] 避免浮点误差
            double angle = std::acos(cosTheta) * 180.0 / CV_PI; // 转换为角度

            if (angle > angleThreshold) { // 超过目标角度的拐点
                cornerPoints.push_back(B);
            }
        }
    }
    return cornerPoints;
}


void drawContour(cv::Mat background, const std::vector<cv::Point2f>& contour, cv::Scalar scalar, bool markPoint) {
    for (size_t i = 0; i < contour.size() - 1; ++i) {
        const cv::Point& p1 = contour[i];
        const cv::Point& p2 = contour[(i + 1) % contour.size()];
        cv::line(background, p1, p2, scalar, 1);

        if(markPoint) cv::circle(background, p1, 2, cv::Scalar(0, 0, 255), cv::FILLED);
    }
}

void drawContours(cv::Mat background, const std::vector<std::vector<cv::Point2f> >& contours, cv::Scalar scalar) {
    for (const auto& contour : contours) {
        drawContour(background, contour, scalar);
    }
}

void moveContour(std::vector<cv::Point2f>& contour, cv::Rect& area, int dx, int dy) {
    for (auto& pt : contour) {
        if (area.contains(pt)) {
            pt.x += dx;
            pt.y += dy;
        }
    }

    area.x += dx;
    area.y += dy;
}

void deleteContour(std::vector<cv::Point2f>& contour, cv::Rect& area) {
    contour.erase(std::remove_if(contour.begin(), contour.end(),
        [&area](const cv::Point2f& pt) {
            return area.contains(pt);
        }),
        contour.end());

    // 确保封闭性：如果首尾点被删了，重新添加
    if (!contour.empty() && contour.back() != contour.front()) {
        contour.push_back(contour.front());
    }
}

// 检查新点是否在两个点之间（在水平或垂直方向上满足中间位置）
bool isPointBetween(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& newPoint) {
    // 检查水平和垂直方向是否满足中间条件
    bool isHorizontal = (p1.x <= newPoint.x && newPoint.x <= p2.x) || (p2.x <= newPoint.x && newPoint.x <= p1.x);
    bool isVertical = (p1.y <= newPoint.y && newPoint.y <= p2.y) || (p2.y <= newPoint.y && newPoint.y <= p1.y);
    return isHorizontal || isVertical;
}

// 在当前轮廓中找到最接近新点的两个相邻点，并将新点插入它们之间
void insertPoint(std::vector<cv::Point2f>& contour, const cv::Point& newPoint) {
    if (contour.empty()) {
        contour.push_back(newPoint);  // 如果轮廓为空，直接插入
        return;
    }

    double minDist = std::numeric_limits<double>::max();
    int insertIndex = -1;

    // 遍历所有相邻的点对，找到满足"中间条件"且距离最小的点对
    for (size_t i = 0; i < contour.size(); ++i) {
        const cv::Point2f& p1 = contour[i];
        const cv::Point2f& p2 = contour[(i + 1) % contour.size()];

        // 检查新点是否在这两个点之间
        if (isPointBetween(p1, p2, newPoint)) {
            double dist1 = distance(p1, newPoint);
            double dist2 = distance(p2, newPoint);
            double totalDist = dist1 + dist2;

            // 更新最小距离和插入位置
            if (totalDist < minDist) {
                minDist = totalDist;
                insertIndex = static_cast<int>(i);
            }
        }
    }

    // 如果找到满足条件的相邻点对，插入新点
    if (insertIndex != -1) {
        contour.insert(contour.begin() + insertIndex + 1, newPoint);
    }
    else {
        // 如果没有找到满足条件的点对，将点添加到末尾
        contour.push_back(newPoint);
    }
}

std::vector<cv::Point2f> smoothContourWithSlidingWindow(const std::vector<cv::Point2f>& contour, int windowSize) {
    std::vector<cv::Point2f> smoothedContour;
    int n = (int)contour.size();

    for (int i = 0; i < n; ++i) {
        int count = 0;
        cv::Point2f sum(0, 0);

        // 滑动窗口内的平均
        for (int j = -windowSize / 2; j <= windowSize / 2; ++j) {
            int idx = (i + j + n) % n; // 环形轮廓，取模处理
            sum += contour[idx];
            ++count;
        }

        smoothedContour.push_back(sum / count);
    }

    // 闭合轮廓：将最后一个点连接到第一个点
    if (!smoothedContour.empty() && smoothedContour.back() != smoothedContour.front()) {
        smoothedContour.push_back(smoothedContour.front());
    }
    return smoothedContour;
}

// 高斯平滑
std::vector<cv::Point2f> gaussianSmooth(const std::vector<cv::Point>& contour, int kernelSize, double sigma) {
    cv::Mat pointsMat((int)contour.size(), 1, CV_32SC2, const_cast<cv::Point*>(contour.data()));
    pointsMat.convertTo(pointsMat, CV_32FC2); // 转换为浮点类型

    // 高斯平滑
    cv::GaussianBlur(pointsMat, pointsMat, cv::Size(kernelSize, kernelSize), sigma);

    std::vector<cv::Point2f> smoothedContour;
    smoothedContour.reserve(pointsMat.rows);  // 预分配空间,减少 push_back 时的内存重新分配

    // 直接访问矩阵数据，避免使用 at() 方法
    const cv::Vec2f* data = pointsMat.ptr<cv::Vec2f>(0);  // 获取指向数据的指针

    for (int i = 0; i < pointsMat.rows; ++i) {
        smoothedContour.push_back(cv::Point2f(data[i][0], data[i][1]));  // 使用直接的内存访问
    }

    // 闭合轮廓：将最后一个点连接到第一个点
    if (!smoothedContour.empty() && smoothedContour.back() != smoothedContour.front()) {
        smoothedContour.push_back(smoothedContour.front());
    }
    return smoothedContour;
}

std::vector<cv::Point2f> gaussianSmooth(const std::vector<cv::Point2f>& contour, int kernelSize, double sigma) {
    int n = (int)contour.size();  // 轮廓的点数
    std::vector<cv::Point2f> smoothedContour(n);

    // 根据点数动态调整高斯核的大小
    int adjustedKernelSize = std::min(kernelSize, n / 2);
    if (adjustedKernelSize % 2 == 0) adjustedKernelSize++;//确保为奇数，否则kernel访问会出界
    int halfKernelSize = adjustedKernelSize / 2;

    // 计算高斯核（环状卷积的权重）
    std::vector<double> kernel(adjustedKernelSize);
    double sum = 0.0f;

    // 计算一维高斯核
    for (int i = -halfKernelSize; i <= halfKernelSize; ++i) {
        kernel[i + halfKernelSize] = exp(-0.5 * (i * i) / (sigma * sigma));
        sum += kernel[i + halfKernelSize];
    }

    // 归一化高斯核
    for (int i = 0; i < adjustedKernelSize; ++i) {
        kernel[i] /= sum;
    }

    // 进行环状卷积
    for (int i = 0; i < n; ++i) {
        double smoothedX = 0.0f;
        double smoothedY = 0.0f;

        // 对每个点进行高斯模糊
        for (int k = -halfKernelSize; k <= halfKernelSize; ++k) {
            int idx = (i + k + n) % n;  // 环状索引，确保首尾相连
            smoothedX += contour[idx].x * kernel[k + halfKernelSize];
            smoothedY += contour[idx].y * kernel[k + halfKernelSize];
        }

        smoothedContour[i] = cv::Point2f((float)smoothedX, (float)smoothedY);
    }

    // 闭合轮廓：将最后一个点连接到第一个点
    if (!smoothedContour.empty() && smoothedContour.back() != smoothedContour.front()) {
        smoothedContour.push_back(smoothedContour.front());
    }
    return smoothedContour;
}

std::vector<cv::Point2f> smoothContourWithBezier(const std::vector<cv::Point2f>& contour, const std::vector<cv::Point2f>& corners, double cornerThreshold, bool enableCorner) {
    if (contour.empty()) return {};

    cv::RotatedRect minRect = cv::minAreaRect(contour);
    unsigned int numPoints = static_cast<unsigned int>(minRect.size.width + minRect.size.height);
    int numDigits = std::to_string(numPoints).length();
    numPoints = numPoints / (numDigits * numDigits / 3.0f);

    int numThreads = std::max(1, static_cast<int>(std::min(numPoints >> 5, std::thread::hardware_concurrency())));
    int segmentSize = std::max(1, static_cast<int>(contour.size() / numThreads));

    std::vector<cv::Point2f> smoothedContour;
    std::vector<std::future<std::vector<cv::Point2f>>> futures;

    for (int t = 0; t < numThreads; ++t) {
        int start = t * segmentSize;
        int end = (t == numThreads - 1) ? static_cast<int>(contour.size()) : std::min((t + 1) * segmentSize, static_cast<int>(contour.size()));

        if (start >= contour.size()) break;

        futures.push_back(std::async(std::launch::async, [start, end, &contour, numPoints, numThreads, &corners, cornerThreshold]() {
            std::vector<cv::Point2f> segment;

            auto bezierPoint = [](const std::vector<cv::Point2f>& points, double t) -> cv::Point2f {
                std::vector<cv::Point2f> temp(points.begin(), points.end());
                size_t n = points.size();
                for (size_t k = 1; k < n; ++k) {
                    for (size_t i = 0; i < n - k; ++i) {
                        temp[i] = temp[i] * (1.0 - t) + temp[i + 1] * t;
                    }
                }
                return temp[0];
            };

            // 动态调整参数 t 的间隔，同时标记靠近拐点的点
            std::vector<double> tValues;
            for (int i = 0; i < end - start; ++i) {
                for (const auto& corner : corners) {
                    if (cv::norm(corner - contour[start + i]) < cornerThreshold) {
                        tValues.push_back(static_cast<double>(i) / (end - start - 1));
                        break; // 一旦检测到靠近某个拐点，就退出循环
                    }
                }
            }

            // 添加均匀分布的 t 值
            int localNumPoints = std::max(1U, numPoints / numThreads);
            for (int i = 0; i < localNumPoints; ++i) {
                double t = static_cast<double>(i) / (localNumPoints - 1);
                tValues.push_back(t);
            }

            // 去重并排序 t 值
            std::sort(tValues.begin(), tValues.end());
            tValues.erase(std::unique(tValues.begin(), tValues.end()), tValues.end());

            // 生成贝塞尔曲线点
            segment.reserve(tValues.size());
            const std::vector<cv::Point2f> segmentContour(contour.begin() + start, contour.begin() + end);
            for (const auto& t : tValues) {
                segment.push_back(bezierPoint(segmentContour, t));
            }

            return segment;
            }));
    }

    for (auto& f : futures) {
        auto segment = f.get();
        smoothedContour.insert(smoothedContour.end(), segment.begin(), segment.end());
    }

    // 确保闭合轮廓
    if (!smoothedContour.empty() && smoothedContour.back() != smoothedContour.front()) {
        smoothedContour.push_back(smoothedContour.front());
    }

    // 如果没有传入拐点，则自动检测并递归调用
    if (enableCorner && corners.empty()) {
        std::vector<cv::Point2f> cornerPoints = findCornerPoints(smoothedContour);
        if (!cornerPoints.empty()) {
            smoothedContour = smoothContourWithBezier(contour, cornerPoints, cornerThreshold);
        }
    }
    return smoothedContour;
}

std::vector<cv::Point2f> smoothContourWithBezier(const std::vector<cv::Point2f>& contour, const cv::Rect& area) {
    if (contour.empty()) return {};

    // 1. 遍历轮廓，找出所有连续的被框选区域，保存为 [startIdx, endIdx] 对
    std::vector<std::pair<int, int>> segments;
    bool inSegment = false;
    int segStart = -1;
    for (int i = 0; i < static_cast<int>(contour.size()); i++) {
        if (area.contains(contour[i])) {
            if (!inSegment) {
                inSegment = true;
                segStart = i; // 记录当前连续区域起始索引
            }
        }
        else {
            if (inSegment) {
                // 当前连续区域结束
                segments.push_back({ segStart, i - 1 });
                inSegment = false;
            }
        }
    }
    // 如果最后一直在选区内，则补充最后一个区域
    if (inSegment) {
        segments.push_back({ segStart, static_cast<int>(contour.size()) - 1 });
    }

    // 如果没有任何点被框选，则直接返回原轮廓
    if (segments.empty()) return contour;

    // 贝塞尔插值函数（递归迭代方式）
    auto bezierPoint = [](const std::vector<cv::Point2f>& points, double t) -> cv::Point2f {
        size_t n = points.size();
        std::vector<cv::Point2f> temp(points.begin(), points.end());
        for (size_t k = 1; k < n; ++k) {
            for (size_t i = 0; i < n - k; ++i) {
                temp[i] = temp[i] * (1.0 - t) + temp[i + 1] * t;
            }
        }
        return temp[0];
    };

    // 2. 构造最终轮廓，将各个区段依次处理
    std::vector<cv::Point2f> finalContour;
    int currentIndex = 0;
    for (const auto& seg : segments) {
        int segStart = seg.first;
        int segEnd = seg.second;
        // 将 currentIndex 到 segStart-1 的点原样保留
        for (int i = currentIndex; i < segStart; i++) {
            finalContour.push_back(contour[i]);
        }

        // 处理选区内的点：提取连续区域并平滑
        std::vector<cv::Point2f> insidePoints(contour.begin() + segStart, contour.begin() + segEnd + 1);
        int numPoints = segEnd - segStart + 1;

        std::vector<cv::Point2f> smoothedSegment;
        for (int i = 0; i < numPoints; ++i) {
            double t = (numPoints > 1) ? static_cast<double>(i) / (numPoints - 1) : 0.0;
            smoothedSegment.push_back(bezierPoint(insidePoints, t));
        }
        // 将平滑后的选区插入最终轮廓
        finalContour.insert(finalContour.end(), smoothedSegment.begin(), smoothedSegment.end());
        currentIndex = segEnd + 1;
    }
    // 将 currentIndex 到末尾的部分原样保留
    for (int i = currentIndex; i < static_cast<int>(contour.size()); i++) {
        finalContour.push_back(contour[i]);
    }

    // 如果原轮廓是闭合的，则保证平滑后轮廓也闭合
    if (!finalContour.empty() && finalContour.front() != finalContour.back()) {
        finalContour.push_back(finalContour.front());
    }

    return finalContour;
}

//双边滤波
std::vector<cv::Point2f> smoothContourWithBilateral(const std::vector<cv::Point2f>& contour, int windowSize, double spatialSigma, double intensitySigma) {
    std::vector<cv::Point2f> smoothedContour(contour.size());
    int n = (int)contour.size();

    for (int i = 0; i < n; ++i) {
        cv::Point2f smoothedPoint(0, 0);
        double totalWeight = 0.0;

        for (int j = -windowSize; j <= windowSize; ++j) {
            int idx = (i + j + n) % n; // 循环处理轮廓

            // 计算空间权重和强度权重
            double spatialWeight = exp(-0.5 * (j * j) / (spatialSigma * spatialSigma));
            double intensityWeight = exp(-0.5 * (cv::norm(contour[i] - contour[idx]) * cv::norm(contour[i] - contour[idx])) / (intensitySigma * intensitySigma));

            double weight = spatialWeight * intensityWeight;
            smoothedPoint += contour[idx] * weight;
            totalWeight += weight;
        }

        smoothedContour[i] = smoothedPoint / totalWeight;
    }

    // 闭合轮廓：将最后一个点连接到第一个点
    if (!smoothedContour.empty() && smoothedContour.back() != smoothedContour.front()) {
        smoothedContour.push_back(smoothedContour.front());
    }
    return smoothedContour;
}