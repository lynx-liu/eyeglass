﻿///////////////////////////////////////////////////
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
const double epsilon = 1e-6;

Detector::Detector()
{
    reset();
}

Detector::~Detector()
{
    reset();
}

void Detector::reset(cv::Rect rect, double pxToMm)
{
    editArea = rect;
    PxToMM = pxToMm;
    onlyContour = false;
    isPreview = false;

    isEditSelectArea = false;
    selectRect = cv::Rect(0, 0, 0, 0);
    scale = 1.0;
    rotationCenter = cv::Point2f(editArea.size() / 2);

    currentContour.clear();
    eyeglassContours.clear();
}

std::vector<cv::Point2f> Detector::findContourInRect(cv::Mat frame, double clipLimit, int medianBlurKSize, int morphKSize, cv::Mat background, cv::Rect roi) {
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

    if (roi.empty()) {
        int offset = 10;
        roi = cv::Rect(offset, offset, gray.cols - offset * 2, gray.rows - offset * 2);
    }

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gray(roi), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE, roi.tl());

    //把灰度图缩小后贴到背景右上角展示
    if (m_register.showQQ()) {
        cv::resize(gray, gray, cv::Size(cvRound(gray.cols / 2.0), cvRound(gray.rows / 2.0)), 0, 0, cv::INTER_LINEAR);
        cv::cvtColor(gray, background(cv::Rect(background.cols - gray.cols, 0, gray.cols, gray.rows)), cv::COLOR_GRAY2BGR);
    }

    if (!contours.empty()) {
        int maxExternal = findMaxContourId(contours);
        return smoothContourWithSlidingWindow(smoothContourWithBezier(gaussianSmooth(smoothContourWithBilateral(convertToPoint2f(contours[maxExternal])))));
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
        return smoothContourWithSlidingWindow(smoothContourWithBezier(gaussianSmooth(smoothContourWithBilateral(convertToPoint2f(contours[maxExternal])))));
    }
    return {};
}

void Detector::detect(cv::Mat frame, double clipLimit, int medianBlurKSize, int morphKSize, cv::Mat background)
{
    if (eyeglassContours.empty()) {
        if (selectRect.empty()) {
            currentContour = findContourInRect(frame, clipLimit, medianBlurKSize, morphKSize, background);
        }
        else {
            currentContour = findContourInRect(frame, clipLimit, medianBlurKSize, morphKSize, background, selectRect);
        }
        boundRect = boundingRect(currentContour);
        pupilCenter = cv::Point(boundRect.x + (boundRect.width >> 1), boundRect.y + (boundRect.height >> 1));
    }
    else {
        drawContours(frame, eyeglassContours, cv::Scalar(0, 255, 0));

        if (selectRect.empty()) {
            std::vector<cv::Point2f> contour = scaleContour(eyeglassContours.back(), 7, computeContourCenter(eyeglassContours.back()));
            if (!contour.empty()) {
                currentContour = findContourInMask(frame, clipLimit, medianBlurKSize, morphKSize, contour, background);
            }
            else {
                currentContour.clear();
            }
        }
        else {
            currentContour = findContourInRect(frame, clipLimit, medianBlurKSize, morphKSize, background, selectRect);
        }
    }

    drawFrame(frame, background);
}

void Detector::drawFrame(cv::Mat frame, cv::Mat background, bool mark)
{
    fps_.tic();

    cv::Mat roi = background(cv::Rect(0, 0, frame.cols, frame.rows));
    if (onlyContour && !isPreview) frame.setTo(cv::Scalar(0));//缩放轮廓模式不显示图像

    if (frame.channels() > 1) {
        frame.copyTo(roi);
    }
    else {
        cv::cvtColor(frame, roi, cv::COLOR_GRAY2BGR);
    }

    if (mark) {
        if (!boundRect.empty()) {
            cv::rectangle(roi, boundRect, cv::Scalar(255, 255, 0), 1);

            cv::Point center = cv::Point(boundRect.x + (boundRect.width >> 1), boundRect.y + (boundRect.height >> 1));
            cv::line(roi, center, cv::Point(center.x, boundRect.y), cv::Scalar(255, 255, 0), 1);
            cv::line(roi, cv::Point(boundRect.x, center.y), center, cv::Scalar(255, 255, 0), 1);

            std::ostringstream ossWidth;//格式化输出
            ossWidth << std::fixed << std::setprecision(2) << (boundRect.width / PxToMM);
            cv::putText(roi, ossWidth.str(), cv::Point(center.x - 25, std::min(frame.rows - 25, boundRect.y + boundRect.height + 25)), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 128, 0));

            std::ostringstream ossHeight;
            ossHeight << std::fixed << std::setprecision(2) << (boundRect.height / PxToMM);
            cv::putText(roi, ossHeight.str(), cv::Point(std::min(frame.cols - 75, boundRect.x + boundRect.width - 25), center.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 128, 0));

            int offset = 50;
            cv::line(roi, cv::Point(pupilCenter.x-offset, pupilCenter.y), cv::Point(pupilCenter.x+offset, pupilCenter.y), cv::Scalar(0, 255, 255), 1);
            cv::line(roi, cv::Point(pupilCenter.x, pupilCenter.y-offset), cv::Point(pupilCenter.x, pupilCenter.y+offset), cv::Scalar(0, 255, 255), 1);
        }
    }

    if (!eyeglassContours.empty()) drawContours(roi, eyeglassContours, cv::Scalar(0, 255, 0));
    if (!currentContour.empty()) drawContour(roi, currentContour, cv::Scalar(255, 0, 0), true);

    cv::rectangle(roi, selectRect, cv::Scalar(255, 0, 255), 2);

    if (m_register.showQQ()) {
        putText(roi, m_register.getMark(), cv::Point(frame.cols / 3, frame.rows / 2), cv::FONT_HERSHEY_COMPLEX, 1.2, cv::Scalar(0, 0, 255), 2);

        fps_.toc();
        cv::putText(roi, fps_.toString(), cv::Point(0, frame.rows - 3), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 128, 255));
    }
}

cv::Mat Detector::rotate(cv::Mat frame, double angle) {
    if (std::abs(angle)< epsilon && std::abs(scale - 1.0)< epsilon) {
        rotationCenter = cv::Point2f(frame.size() / 2);
        return frame;
    }

    cv::Mat rotateMat = getRotationMatrix2D(rotationCenter, angle, scale);
    warpAffine(frame, frame, rotateMat, frame.size());
    return frame;
}

bool Detector::findNext() {
    if (!currentContour.empty() && std::abs(scale - 1.0)< epsilon) {
        eyeglassContours.emplace_back(currentContour);
        currentContour.clear();
        return true;
    }
    return false;
}

bool Detector::scaleCurrentContour(int N) {
    if (std::abs(scale - 1.0) >= epsilon) return false;
    currentContour = scaleContour(currentContour, -N, computeContourCenter(currentContour));
    boundRect = boundingRect(currentContour);
    return true;
}

void Detector::setOnlyContour(bool state) {
    onlyContour = state;
}

void Detector::setPreview(bool preview) {
    isPreview = preview;
}

bool Detector::saveToDxf(std::string filename) {
    if (!m_register.isRegisted()) return false;

    std::vector<std::vector<cv::Point2f>> contours = eyeglassContours;
    contours.emplace_back(currentContour);
    Dxf::SaveContoursToFile(contours, filename, 1.0/PxToMM);
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

    case 'f':
    case 'F':
        currentContour = smoothContourWithBezier(currentContour, selectRect);
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
        mousePoint = cv::Point(x, y);
        if (onlyContour) {//only scale contour
            int N = flags < 0 ? 1 : -1;
            double scaleN = computeScale(currentContour, N);
            if (scaleN > 0) {
                cv::Point2f center = computeContourCenter(currentContour);
                currentContour = scaleContour(currentContour, scaleN, center, getAxis(center, mousePoint));
                boundRect = boundingRect(currentContour);
            }
        }
        else {
            int N = flags < 0 ? 24 : -24;
            double scaleN = computeScale(currentContour, N);
            if (scaleN > 0) {
                if (std::abs(scale - 1.0) < epsilon && rotationCenter.x == editArea.width / 2 && rotationCenter.y == editArea.height / 2) {
                    rotationCenter = mousePoint;// 以缩放开始时的鼠标位置为缩放中心
                }

                currentContour = scaleContour(currentContour, scaleN, rotationCenter);
                scale *= scaleN;//相对原始图的累积缩放因子
                return true;
            }
        }
    }
        break;

    case cv::EVENT_MBUTTONDBLCLK:
        currentContour = scaleContour(currentContour, 1.0/scale, rotationCenter);
        scale = 1.0;
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

double Detector::getPupilWidth() {
    return (boundRect.x + boundRect.width - pupilCenter.x) / PxToMM;
}

double Detector::getPupilHeight() {
    return (boundRect.y + boundRect.height - pupilCenter.y) / PxToMM;
}

void Detector::setPupilWidth(double pupilWidth) {
    pupilCenter.x = boundRect.x + boundRect.width - (int)(pupilWidth * PxToMM);
}

void Detector::setPupilHeight(double pupilHeight) {
    pupilCenter.y = boundRect.y + boundRect.height - (int)(pupilHeight * PxToMM);
}