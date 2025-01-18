///////////////////////////////////////////////////
//				date: 2025.01.10
//				author: ������  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include "stdafx.h"
#include "Contour.h"
#include <future>

// ��������֮���ŷ����þ���
inline double distance(const cv::Point2f& pt1, const cv::Point2f& pt2) {
    return std::sqrt(std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2));
}

// �������壺�� std::vector<cv::Point> ת��Ϊ std::vector<cv::Point2f>
std::vector<cv::Point2f> convertToPoint2f(const std::vector<cv::Point>& contour) {
    std::vector<cv::Point2f> result;
    result.reserve(contour.size()); // Ԥ����ռ����������

    for (const auto& point : contour) {
        result.emplace_back(point);
    }

    return result;
}

// ʹ�õ�ƫ��ֱ����������N�����ص㣬N<0Ϊ�Ŵ�
std::vector<cv::Point> scaleContour(const std::vector<cv::Point2f>& contour, int N) {
    cv::Point2f center(0, 0);
    for (const auto& point : contour) {
        center += point;
    }
    center /= (int)contour.size();

    std::vector<cv::Point> scaleContour;
    scaleContour.reserve(contour.size());
    for (const auto& point : contour) {
        cv::Point2f direction = point - center;
        float length = distance(point, center);
        if (length > 0) direction /= length;

        scaleContour.push_back(point - direction * N);
    }
    return scaleContour;
}

int findMaxContourId(const std::vector<std::vector<cv::Point> >& contours)
{
    int index = 0;
    int maxArea = cv::minAreaRect(contours[index]).boundingRect().area();
    for (size_t i = 1; i < contours.size(); i++) {
        int area = cv::minAreaRect(contours[i]).boundingRect().area();
        if (area > maxArea) {
            index = i;
            maxArea = area;
        }
    }
    return index;
}

void drawContour(cv::Mat background, const std::vector<cv::Point2f>& contour, cv::Scalar scalar, bool markPoint) {
    for (size_t i = 0; i < contour.size() - 1; ++i) {
        const cv::Point& p1 = contour[i];
        const cv::Point& p2 = contour[(i + 1) % contour.size()];
        cv::line(background, p1, p2, scalar, 2);

        if(markPoint) cv::circle(background, p1, 3, cv::Scalar(0, 0, 255), cv::FILLED);
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
        [&area](const cv::Point& pt) {
            return area.contains(pt);
        }),
        contour.end());
}

// ����µ��Ƿ���������֮�䣨��ˮƽ��ֱ�����������м�λ�ã�
bool isPointBetween(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& newPoint) {
    // ���ˮƽ�ʹ�ֱ�����Ƿ������м�����
    bool isHorizontal = (p1.x <= newPoint.x && newPoint.x <= p2.x) || (p2.x <= newPoint.x && newPoint.x <= p1.x);
    bool isVertical = (p1.y <= newPoint.y && newPoint.y <= p2.y) || (p2.y <= newPoint.y && newPoint.y <= p1.y);
    return isHorizontal || isVertical;
}

// �ڵ�ǰ�������ҵ���ӽ��µ���������ڵ㣬�����µ��������֮��
void insertPoint(std::vector<cv::Point2f>& contour, const cv::Point& newPoint) {
    if (contour.empty()) {
        contour.push_back(newPoint);  // �������Ϊ�գ�ֱ�Ӳ���
        return;
    }

    double minDist = std::numeric_limits<double>::max();
    int insertIndex = -1;

    // �����������ڵĵ�ԣ��ҵ�����"�м�����"�Ҿ�����С�ĵ��
    for (size_t i = 0; i < contour.size() - 1; ++i) {
        const cv::Point2f& p1 = contour[i];
        const cv::Point2f& p2 = contour[i + 1];

        // ����µ��Ƿ�����������֮��
        if (isPointBetween(p1, p2, newPoint)) {
            double dist1 = distance(p1, newPoint);
            double dist2 = distance(p2, newPoint);
            double totalDist = dist1 + dist2;

            // ������С����Ͳ���λ��
            if (totalDist < minDist) {
                minDist = totalDist;
                insertIndex = static_cast<int>(i);
            }
        }
    }

    // ����ҵ��������������ڵ�ԣ������µ�
    if (insertIndex != -1) {
        contour.insert(contour.begin() + insertIndex + 1, newPoint);
    }
    else {
        // ���û���ҵ����������ĵ�ԣ�������ӵ�ĩβ
        contour.push_back(newPoint);
    }
}

std::vector<cv::Point2f> smoothContourWithSlidingWindow(const std::vector<cv::Point2f>& contour, int windowSize) {
    std::vector<cv::Point2f> smoothedContour;
    int n = contour.size();

    for (int i = 0; i < n; ++i) {
        int count = 0;
        cv::Point2f sum(0, 0);

        // ���������ڵ�ƽ��
        for (int j = -windowSize / 2; j <= windowSize / 2; ++j) {
            int idx = (i + j + n) % n; // ����������ȡģ����
            sum += contour[idx];
            ++count;
        }

        smoothedContour.push_back(sum / count);
    }

    // �պ������������һ�������ӵ���һ����
    smoothedContour.push_back(smoothedContour[0]);
    return smoothedContour;
}

// ��˹ƽ��
std::vector<cv::Point2f> gaussianSmooth(const std::vector<cv::Point>& contour, int kernelSize, double sigma) {
    cv::Mat pointsMat(contour.size(), 1, CV_32SC2, const_cast<cv::Point*>(contour.data()));
    pointsMat.convertTo(pointsMat, CV_32FC2); // ת��Ϊ��������

    // ��˹ƽ��
    cv::GaussianBlur(pointsMat, pointsMat, cv::Size(kernelSize, kernelSize), sigma);

    std::vector<cv::Point2f> smoothedContour;
    smoothedContour.reserve(pointsMat.rows);  // Ԥ����ռ�,���� push_back ʱ���ڴ����·���

    // ֱ�ӷ��ʾ������ݣ�����ʹ�� at() ����
    const cv::Vec2f* data = pointsMat.ptr<cv::Vec2f>(0);  // ��ȡָ�����ݵ�ָ��

    for (int i = 0; i < pointsMat.rows; ++i) {
        smoothedContour.push_back(cv::Point2f(data[i][0], data[i][1]));  // ʹ��ֱ�ӵ��ڴ����
    }

    // �պ������������һ�������ӵ���һ����
    smoothedContour.push_back(smoothedContour[0]);
    return smoothedContour;
}

std::vector<cv::Point2f> gaussianSmooth(const std::vector<cv::Point2f>& contour, int kernelSize, double sigma) {
    cv::Mat pointsMat(contour);

    // ��˹ƽ��
    cv::GaussianBlur(pointsMat, pointsMat, cv::Size(kernelSize, kernelSize), sigma);

    std::vector<cv::Point2f> smoothedContour;
    smoothedContour.reserve(pointsMat.rows);  // Ԥ����ռ�,���� push_back ʱ���ڴ����·���

    // ֱ�ӷ��ʾ������ݣ�����ʹ�� at() ����
    const cv::Vec2f* data = pointsMat.ptr<cv::Vec2f>(0);  // ��ȡָ�����ݵ�ָ��

    for (int i = 0; i < pointsMat.rows; ++i) {
        smoothedContour.push_back(cv::Point2f(data[i][0], data[i][1]));  // ʹ��ֱ�ӵ��ڴ����
    }

    // �պ������������һ�������ӵ���һ����
    smoothedContour.push_back(smoothedContour[0]);
    return smoothedContour;
}

std::vector<cv::Point2f> smoothContourWithBezier(const std::vector<cv::Point2f>& contour, int numPoints, int numThreads) {
    std::vector<cv::Point2f> smoothedContour;
    std::vector<std::future<std::vector<cv::Point2f>>> futures;

    // �ֶμ���
    int segmentSize = contour.size() / numThreads;
    for (int t = 0; t < numThreads; ++t) {
        int start = t * segmentSize;
        int end = (t == numThreads - 1) ? contour.size() : (t + 1) * segmentSize;

        // ��Ӳ���ı���
        futures.push_back(std::async(std::launch::async, [start, end, &contour, numPoints, numThreads]() {
            std::vector<cv::Point2f> segment;
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

            for (int i = 0; i < numPoints / numThreads; ++i) {
                double t = static_cast<double>(i) / (numPoints / numThreads - 1);
                segment.push_back(bezierPoint(std::vector<cv::Point2f>(contour.begin() + start, contour.begin() + end), t));
            }
            return segment;
            }));
    }

    // �ϲ����
    for (auto& f : futures) {
        auto segment = f.get();
        smoothedContour.insert(smoothedContour.end(), segment.begin(), segment.end());
    }

    // �պ������������һ�������ӵ���һ����
    smoothedContour.push_back(smoothedContour[0]);
    return smoothedContour;
}

//˫���˲�
std::vector<cv::Point2f> smoothContourWithBilateral(const std::vector<cv::Point2f>& contour, int windowSize, double spatialSigma, double intensitySigma) {
    std::vector<cv::Point2f> smoothedContour(contour.size());
    int n = contour.size();

    for (int i = 0; i < n; ++i) {
        cv::Point2f smoothedPoint(0, 0);
        double totalWeight = 0.0;

        for (int j = -windowSize; j <= windowSize; ++j) {
            int idx = (i + j + n) % n; // ѭ����������

            // ����ռ�Ȩ�غ�ǿ��Ȩ��
            double spatialWeight = exp(-0.5 * (j * j) / (spatialSigma * spatialSigma));
            double intensityWeight = exp(-0.5 * (cv::norm(contour[i] - contour[idx]) * cv::norm(contour[i] - contour[idx])) / (intensitySigma * intensitySigma));

            double weight = spatialWeight * intensityWeight;
            smoothedPoint += contour[idx] * weight;
            totalWeight += weight;
        }

        smoothedContour[i] = smoothedPoint / totalWeight;
    }

    // �պ������������һ�������ӵ���һ����
    smoothedContour.push_back(smoothedContour[0]);
    return smoothedContour;
}