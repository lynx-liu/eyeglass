///////////////////////////////////////////////////
//				date: 2022.06.30
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include "dxf.h"
#include "debug.h"
#include "fps.hpp"
#include "Register.h"
#include "Contour.h"
#include "detector.h"

#define KEY_LEFT	0x250000	// 向左
#define KEY_TOP		0x260000	// 向上
#define KEY_RIGHT	0x270000	// 向右
#define KEY_BOTTOM	0x280000	// 向下
#define KEY_DEL		0x2E0000	// 删除

Register m_register;
FPS fps_ = FPS();

Detector::Detector()
{
    reset();
}

Detector::~Detector()
{
    reset();
}

void Detector::reset(cv::Rect rect)
{
    editArea = rect;

    isEditSelectArea = false;
    selectRect = cv::Rect(0, 0, 0, 0);

    currentContour.clear();
    eyeglassContours.clear();
}

std::vector<cv::Point2f> Detector::findExternalContour(cv::Mat frame, double clipLimit, int medianBlurKSize, int morphKSize, cv::Mat background) {
    cv::Mat gray;
    if (frame.channels() > 1) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        frame.copyTo(gray);
    }
    
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clipLimit, cv::Size(5, 5));
    clahe->apply(gray, gray);

    medianBlur(gray, gray, medianBlurKSize);
    blur(gray, gray, cv::Size(3, 3));
    stackBlur(gray, gray, cv::Size(5, 5));
    GaussianBlur(gray, gray, cv::Size(3, 3), 0);

    cv::Mat input = cv::Mat(3, 3, CV_8UC1);
    erode(gray, gray, input);//腐蚀
    dilate(gray, gray, input);//膨胀

    Canny(gray, gray, 1, 180, 3);

    cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphKSize, morphKSize));
    cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, morphKernel);// 形态学闭运算

    int offset = 10;
    cv::Rect roi = cv::Rect(offset, offset, gray.cols - offset * 2, gray.rows - offset * 2);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gray(roi), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,cv::Point(offset,offset));

    //把灰度图缩小后贴到背景右上角展示
    cv::resize(gray, gray, cv::Size(cvRound(gray.cols / 2.0), cvRound(gray.rows / 2.0)), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(gray, background(cv::Rect(background.cols - gray.cols, 0, gray.cols, gray.rows)), cv::COLOR_GRAY2BGR);

    if (!contours.empty()) {
        int maxExternal = findMaxContourId(contours);
        return smoothContourWithBezier(gaussianSmooth(smoothContourWithBilateral(convertToPoint2f(contours[maxExternal]))));
    }
    return {};
}

std::vector<cv::Point2f> Detector::findContourInMask(cv::Mat frame, double clipLimit, int medianBlurKSize, int morphKSize, const std::vector<cv::Point2f>& contour, cv::Mat background) {
    cv::Mat gray;
    if (frame.channels() > 1) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
    else {
        frame.copyTo(gray);
    }

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clipLimit, cv::Size(5, 5));
    clahe->apply(gray, gray);

    medianBlur(gray, gray, medianBlurKSize);
    blur(gray, gray, cv::Size(3, 3));
    stackBlur(gray, gray, cv::Size(5, 5));
    GaussianBlur(gray, gray, cv::Size(3, 3), 0);

    cv::Mat input = cv::Mat(3, 3, CV_8UC1);
    erode(gray, gray, input);//腐蚀
    dilate(gray, gray, input);//膨胀

    Canny(gray, gray, 1, 150, 3, true);

    cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphKSize, morphKSize));
    cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, morphKernel);// 形态学闭运算

    // 创建一个与原图像大小相同的掩码图像
    cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);

    // 在掩码图像上绘制并填充给定的轮廓
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{convertToPoint(contour)}, -1, cv::Scalar(255), cv::FILLED);

    // 应用掩码到原图像，只保留轮廓范围内的区域
    gray.setTo(cv::Scalar(0), ~mask);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    //把灰度图缩小后贴到背景右上角展示
    cv::resize(gray, gray, cv::Size(cvRound(gray.cols / 2.0), cvRound(gray.rows / 2.0)), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(gray, background(cv::Rect(background.cols - gray.cols, 0, gray.cols, gray.rows)), cv::COLOR_GRAY2BGR);

    if (!contours.empty()) {
        int maxExternal = findMaxContourId(contours);
        return smoothContourWithBezier(gaussianSmooth(smoothContourWithBilateral(convertToPoint2f(contours[maxExternal]))));
    }
    return {};
}

void Detector::detect(cv::Mat frame, double clipLimit, int medianBlurKSize, int morphKSize, cv::Mat background)
{
    if (eyeglassContours.empty()) {
        currentContour = findExternalContour(frame, clipLimit, medianBlurKSize, morphKSize, background);
        boundRect = boundingRect(currentContour);
    }
    else {
        drawContours(frame, eyeglassContours, cv::Scalar(0, 255, 0));

        std::vector<cv::Point2f> contour = scaleContour(eyeglassContours.back(), 7);
        if (!contour.empty()) {
            currentContour = findContourInMask(frame, clipLimit, medianBlurKSize, morphKSize, contour, background);
        }
        else {
            currentContour.clear();
        }
    }

    drawFrame(frame, background);
}

void Detector::drawFrame(cv::Mat frame, cv::Mat background, bool mark)
{
    fps_.tic();

    if (frame.channels() > 1) {
        frame.copyTo(background(cv::Rect(0, 0, frame.cols, frame.rows)));
    }
    else {
        cv::cvtColor(frame, background(cv::Rect(0, 0, frame.cols, frame.rows)), cv::COLOR_GRAY2BGR);
    }

    if (!eyeglassContours.empty()) drawContours(background, eyeglassContours, cv::Scalar(0, 255, 0));
    if (!currentContour.empty()) drawContour(background, currentContour, cv::Scalar(255, 0, 0), true);
    if (mark) {
        if (!boundRect.empty()) {
            cv::rectangle(background, boundRect, cv::Scalar(255, 255, 0), 2);

            cv::Point center = cv::Point(boundRect.x + (boundRect.width >> 1), boundRect.y + (boundRect.height >> 1));
            cv::line(background, center, cv::Point(center.x, boundRect.y), cv::Scalar(255, 255, 0), 2);
            cv::line(background, cv::Point(boundRect.x, center.y), center, cv::Scalar(255, 255, 0), 2);
            cv::putText(background, std::to_string(boundRect.width), cv::Point(center.x - 25, std::min(frame.rows - 25, boundRect.y + boundRect.height + 25)), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 128, 0));
            cv::putText(background, std::to_string(boundRect.height), cv::Point(std::min(frame.cols - 75, boundRect.x + boundRect.width - 25), center.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 128, 0));
        }
    }

    cv::rectangle(background, selectRect, cv::Scalar(255, 0, 255), 2);

    if (m_register.showQQ()) putText(background, m_register.getMark(), cv::Point(frame.cols / 3, frame.rows / 2), cv::FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(0, 0, 255), 2);

    fps_.toc();
    cv::putText(background, fps_.toString(), cv::Point(0, frame.rows - 3), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 128, 255));
}

cv::Mat Detector::rotate(cv::Mat frame, int angle) {
    if (angle == 0) return frame;

    cv::Point2f center = cv::Point2f(frame.cols / 2.0f, frame.rows / 2.0f);//旋转中心
    cv::Mat rotateMat = getRotationMatrix2D(center, angle, 1.0);
    warpAffine(frame, frame, rotateMat, frame.size());
    return frame;
}

void Detector::findNext() {
    if (!currentContour.empty()) {
        eyeglassContours.emplace_back(currentContour);
        currentContour.clear();
    }
}

bool Detector::saveToDxf(std::string filename) {
    if (!m_register.isRegisted()) return false;

    std::vector<std::vector<cv::Point2f>> contours = eyeglassContours;
    contours.emplace_back(currentContour);
    Dxf::SaveContoursToFile(contours, filename);
    return true;
}

bool Detector::onKey(int key) {
    switch (key) {
    case KEY_LEFT:
        moveContour(currentContour, selectRect, -1, 0);
        return true;

    case KEY_RIGHT:
        moveContour(currentContour, selectRect, 1, 0);
        return true;

    case KEY_TOP:
        moveContour(currentContour, selectRect, 0, -1);
        return true;

    case KEY_BOTTOM:
        moveContour(currentContour, selectRect, 0, 1);
        return true;

    case KEY_DEL:
        deleteContour(currentContour, selectRect);
        return true;

    case 'i':
    case 'I':
        insertPoint(currentContour, mousePoint);
        return true;

    case 'r':
    case 'R':
        if (eyeglassContours.empty())
            return true;
        break;
    }
    return false;
}

bool Detector::onMouse(int event, int x, int y, int flags) {
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:
        mousePoint = cv::Point(x, y);
        isEditSelectArea = true;
        selectRect = cv::Rect(x, y, 0, 0);
            break;

    case cv::EVENT_MOUSEMOVE:
        if (isEditSelectArea) {
            int rectX = std::min(mousePoint.x, x);
            int rectY = std::min(mousePoint.y, y);
            int rectWidth = std::abs(x - mousePoint.x);
            int rectHeight = std::abs(y - mousePoint.y);

            selectRect = cv::Rect(rectX, rectY, rectWidth, rectHeight);
        }
        else {
            mousePoint = cv::Point(x, y);
        }
        break;

    case cv::EVENT_LBUTTONUP:
        if (isEditSelectArea) {
            isEditSelectArea = false;
            int rectX = std::min(mousePoint.x, x);
            int rectY = std::min(mousePoint.y, y);
            int rectWidth = std::abs(x - mousePoint.x);
            int rectHeight = std::abs(y - mousePoint.y);

            selectRect = cv::Rect(rectX, rectY, rectWidth, rectHeight);
        }
        else {
            mousePoint = cv::Point(x, y);
        }
        break;

    case cv::EVENT_MOUSEWHEEL: {
        currentContour = scaleContour(currentContour, flags>0?1:-1);
        return true;
    }
        break;

    default:
        break;
    }
    return isEditSelectArea;
}

cv::Point Detector::getMousePoint() {
    return mousePoint;
}

cv::Rect Detector::getEditArea() {
    return editArea;
}