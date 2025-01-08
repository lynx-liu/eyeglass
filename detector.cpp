///////////////////////////////////////////////////
//				date: 2022.06.30
//				author: ������  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include "stdafx.h"
#include "dxf.h"
#include "detector.h"

Detector::Detector()
{
    currentContour.clear();
    eyeglassContours.clear();
}

Detector::~Detector()
{
    eyeglassContours.clear();
    currentContour.clear();
}

std::vector<cv::Point> Detector::scaleContour(const std::vector<cv::Point>& contour, int N) {
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


int Detector::findMaxContourId(std::vector<std::vector<cv::Point> > contours)
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

void Detector::drawContour(cv::Mat background, std::vector<cv::Point> contour) {
    for (size_t i = 0; i < contour.size() - 1; ++i) {
        const cv::Point& p1 = contour[i];
        const cv::Point& p2 = contour[(i + 1) % contour.size()];
        cv::line(background, p1, p2, cv::Scalar(255, 0, 0), 2);

        cv::circle(background, p1, 3, cv::Scalar(0, 0, 255), -1);
    }
}

std::vector<cv::Point> smoothContourWithSlidingWindow(const std::vector<cv::Point>& contour, int windowSize = 5) {
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
std::vector<cv::Point> gaussianSmooth(const std::vector<cv::Point>& contour, int kernelSize = 5, double sigma = 1.0) {
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

std::vector<cv::Point> smoothContourWithBezier(const std::vector<cv::Point>& contour, int numPoints = 250, int numThreads = 4) {
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


std::vector<cv::Point> Detector::findExternalContour(cv::Mat frame, int medianBlurKSize, int morphKSize, cv::Mat background) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    medianBlur(gray, gray, medianBlurKSize);
    blur(gray, gray, cv::Size(3, 3));
    stackBlur(gray, gray, cv::Size(5, 5));
    GaussianBlur(gray, gray, cv::Size(3, 3), 0);

    cv::Mat input = cv::Mat(3, 3, CV_8UC1);
    erode(gray, gray, input);//��ʴ
    dilate(gray, gray, input);//����

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(5, 5));
    clahe->apply(gray, gray);

    Canny(gray, gray, 1, 180, 3);

    cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphKSize, morphKSize));
    cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, morphKernel);// ��̬ѧ������

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    //�ѻҶ�ͼ��С�������������Ͻ�չʾ
    cv::resize(gray, gray, cv::Size(cvRound(gray.cols / 2.0), cvRound(gray.rows / 2.0)), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(gray, background(cv::Rect(background.cols - gray.cols, 0, gray.cols, gray.rows)), cv::COLOR_GRAY2BGR);

    if (!contours.empty()) {
        int maxExternal = findMaxContourId(contours);
        return smoothContourWithBezier(gaussianSmooth(smoothContourWithSlidingWindow(contours[maxExternal])));
    }
    return {};
}

std::vector<cv::Point> Detector::findContourInMask(cv::Mat frame, int medianBlurKSize, int morphKSize, const std::vector<cv::Point>& contour, cv::Mat background) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    medianBlur(gray, gray, medianBlurKSize);
    GaussianBlur(gray, gray, cv::Size(3, 3), 0);

    cv::Mat input = cv::Mat(3, 3, CV_8UC1);
    erode(gray, gray, input);//��ʴ
    dilate(gray, gray, input);//����

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray, gray);

    Canny(gray, gray, 1, 150, 3, true);

    cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphKSize, morphKSize));
    cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, morphKernel);// ��̬ѧ������

    // ����һ����ԭͼ���С��ͬ������ͼ��
    cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);

    // ������ͼ���ϻ��Ʋ�������������
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);

    // Ӧ�����뵽ԭͼ��ֻ����������Χ�ڵ�����
    gray.setTo(cv::Scalar(0), ~mask);

    // ��������
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    //�ѻҶ�ͼ��С�������������Ͻ�չʾ
    cv::resize(gray, gray, cv::Size(cvRound(gray.cols / 2.0), cvRound(gray.rows / 2.0)), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(gray, background(cv::Rect(background.cols - gray.cols, 0, gray.cols, gray.rows)), cv::COLOR_GRAY2BGR);

    if (!contours.empty()) {
        int maxExternal = findMaxContourId(contours);
        return smoothContourWithBezier(gaussianSmooth(smoothContourWithSlidingWindow(contours[maxExternal])));
    }
    return {};
}

void Detector::detect(cv::Mat frame, int medianBlurKSize, int morphKSize, cv::Mat background)
{
    fps_.tic();

    if (eyeglassContours.empty()) {
        currentContour = findExternalContour(frame, medianBlurKSize, morphKSize, background);
    }
    else {
        cv::drawContours(frame, eyeglassContours, -1, cv::Scalar(0, 255, 0), 2);

        std::vector<cv::Point> contour = scaleContour(eyeglassContours.back(), 10);
        if (!contour.empty()) {
            currentContour = findContourInMask(frame, medianBlurKSize, morphKSize, contour, background);
        }
        else {
            currentContour.clear();
        }
    }

    if(!currentContour.empty()) drawContour(frame, currentContour);

    /*
    std::vector<cv::Point> externalContour = findExternalContour(frame, medianBlurKSize, morphKSize);
    if (externalContour.size() > 0) {
        cv::drawContours(frame, std::vector<std::vector<cv::Point>>{externalContour}, -1, cv::Scalar(0, 0, 255), 2);
        
        std::vector<cv::Point> contour = scaleContour(externalContour, 10);
        if (contour.size() > 0) {
            std::vector<cv::Point> innerContour = findContourInMask(frame, 7, 9, contour, background);
            if (innerContour.size()) {
                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{innerContour}, -1, cv::Scalar(255, 0, 0), 2);
            }
            
            cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);//�ŵ����棬����Ӱ�����������
        }
    }*/

    fps_.toc();

    char szText[_MAX_PATH] = { 0 };
    sprintf(szText, "%s", fps_.toString().c_str());
    cv::putText(frame, szText, cv::Point(0, frame.rows - 3), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 128, 255));
    
    frame.copyTo(background(cv::Rect(0, 0, frame.cols, frame.rows)));
}

void Detector::findNext() {
    if (!currentContour.empty()) {
        eyeglassContours.emplace_back(currentContour);
        currentContour.clear();
    }
}

void Detector::saveToDxf(char *filename) {
    std::vector<std::vector<cv::Point>> contours = eyeglassContours;
    contours.emplace_back(currentContour);
    Dxf::SaveContoursToFile(contours, filename);
}