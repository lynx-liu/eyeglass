///////////////////////////////////////////////////
//				date: 2025.01.10
//				author: ������  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include "stdafx.h"
#include "Contour.h"
#include <future>

std::vector<cv::Point> scaleContour(const std::vector<cv::Point>& contour, int N) {
    // ʹ�õ�ƫ��ֱ����������N�����ص㣬N<0Ϊ�Ŵ�
    cv::Point2f center(0, 0);
    for (const auto& point : contour) {
        center.x += point.x;
        center.y += point.y;
    }
    center.x /= contour.size();
    center.y /= contour.size();

    std::vector<cv::Point> scaleContour;
    scaleContour.reserve(contour.size());
    for (const auto& point : contour) {
        cv::Point2f direction = cv::Point2f(point) - center; // ת��Ϊ cv::Point2f
        float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
        if (length > 0) {
            direction.x /= length;
            direction.y /= length;
        }
        cv::Point newPoint = point - cv::Point(cvRound(direction.x * N), cvRound(direction.y * N)); // ת��������
        scaleContour.push_back(newPoint);
    }

    std::vector<cv::Point> hull;
    cv::convexHull(scaleContour, hull);
    return hull;
}

int findMaxContourId(const std::vector<std::vector<cv::Point> >& contours)
{
    int index = 0;
    int maxArea = cv::minAreaRect(contours[index]).boundingRect().area();
    for (int i = 1; i < contours.size(); i++) {
        int area = cv::minAreaRect(contours[i]).boundingRect().area();
        if (area > maxArea) {
            index = i;
            maxArea = area;
        }
    }
    return index;
}

void drawContour(cv::Mat background, const std::vector<cv::Point>& contour) {
    for (size_t i = 0; i < contour.size() - 1; ++i) {
        const cv::Point& p1 = contour[i];
        const cv::Point& p2 = contour[(i + 1) % contour.size()];
        cv::line(background, p1, p2, cv::Scalar(255, 0, 0), 2);

        cv::circle(background, p1, 3, cv::Scalar(0, 0, 255), -1);
    }
}

void moveContour(std::vector<cv::Point>& contour, cv::Rect& area, int dx, int dy) {
    for (auto& pt : contour) {
        if (area.contains(pt)) {
            pt.x += dx;
            pt.y += dy;
        }
    }

    area.x += dx;
    area.y += dy;
}

void deleteContour(std::vector<cv::Point>& contour, cv::Rect& area) {
    contour.erase(std::remove_if(contour.begin(), contour.end(),
        [&area](const cv::Point& pt) {
            return area.contains(pt);
        }),
        contour.end());
}

// ��������֮���ŷ����þ���
double distance(const cv::Point& pt1, const cv::Point& pt2) {
    return std::sqrt(std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2));
}

// ����µ��Ƿ���������֮�䣨��ˮƽ��ֱ�����������м�λ�ã�
bool isPointBetween(const cv::Point& p1, const cv::Point& p2, const cv::Point& newPoint) {
    // ���ˮƽ�ʹ�ֱ�����Ƿ������м�����
    bool isHorizontal = (p1.x <= newPoint.x && newPoint.x <= p2.x) || (p2.x <= newPoint.x && newPoint.x <= p1.x);
    bool isVertical = (p1.y <= newPoint.y && newPoint.y <= p2.y) || (p2.y <= newPoint.y && newPoint.y <= p1.y);
    return isHorizontal || isVertical;
}

// �ڵ�ǰ�������ҵ���ӽ��µ���������ڵ㣬�����µ��������֮��
void insertPoint(std::vector<cv::Point>& contour, const cv::Point& newPoint) {
    if (contour.empty()) {
        contour.push_back(newPoint);  // �������Ϊ�գ�ֱ�Ӳ���
        return;
    }

    double minDist = std::numeric_limits<double>::max();
    int insertIndex = -1;

    // �����������ڵĵ�ԣ��ҵ�����"�м�����"�Ҿ�����С�ĵ��
    for (size_t i = 0; i < contour.size() - 1; ++i) {
        const cv::Point& p1 = contour[i];
        const cv::Point& p2 = contour[i + 1];

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

std::vector<cv::Point> smoothContourWithSlidingWindow(const std::vector<cv::Point>& contour, int windowSize) {
    std::vector<cv::Point> smoothedContour;
    int n = contour.size();

    for (int i = 0; i < n; ++i) {
        int sumX = 0, sumY = 0, count = 0;

        // ���������ڵ�ƽ��
        for (int j = -windowSize; j <= windowSize; ++j) {
            int idx = (i + j + n) % n; // ����������ȡģ����
            sumX += contour[idx].x;
            sumY += contour[idx].y;
            ++count;
        }

        smoothedContour.push_back(cv::Point(sumX / count, sumY / count));
    }

    // �պ������������һ�������ӵ���һ����
    smoothedContour.push_back(smoothedContour[0]);
    return smoothedContour;
}

// ��˹ƽ��
std::vector<cv::Point> gaussianSmooth(const std::vector<cv::Point>& contour, int kernelSize, double sigma) {
    cv::Mat pointsMat(contour.size(), 1, CV_32FC2);
    for (size_t i = 0; i < contour.size(); ++i) {
        pointsMat.at<cv::Vec2f>(i, 0) = cv::Vec2f(contour[i].x, contour[i].y);
    }

    // ��˹ƽ��
    cv::Mat smoothedMat;
    cv::GaussianBlur(pointsMat, smoothedMat, cv::Size(kernelSize, kernelSize), sigma);

    std::vector<cv::Point> smoothedContour;
    for (int i = 0; i < smoothedMat.rows; ++i) {
        cv::Point p(static_cast<int>(std::round(smoothedMat.at<cv::Vec2f>(i, 0)[0])),
            static_cast<int>(std::round(smoothedMat.at<cv::Vec2f>(i, 0)[1])));
        smoothedContour.push_back(p);
    }

    // �պ������������һ�������ӵ���һ����
    smoothedContour.push_back(smoothedContour[0]);
    return smoothedContour;
}

std::vector<cv::Point> smoothContourWithBezier(const std::vector<cv::Point>& contour, int numPoints, int numThreads) {
    std::vector<cv::Point> smoothedContour;
    std::vector<std::future<std::vector<cv::Point>>> futures;

    // �ֶμ���
    int segmentSize = contour.size() / numThreads;
    for (int t = 0; t < numThreads; ++t) {
        int start = t * segmentSize;
        int end = (t == numThreads - 1) ? contour.size() : (t + 1) * segmentSize;

        // ��Ӳ���ı���
        futures.push_back(std::async(std::launch::async, [start, end, &contour, numPoints, numThreads]() {
            std::vector<cv::Point> segment;
            auto bezierPoint = [](const std::vector<cv::Point>& points, double t) -> cv::Point2f {
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
                segment.push_back(bezierPoint(std::vector<cv::Point>(contour.begin() + start, contour.begin() + end), t));
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