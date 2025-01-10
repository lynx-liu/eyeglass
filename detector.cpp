///////////////////////////////////////////////////
//				date: 2022.06.30
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include "stdafx.h"
#include "dxf.h"
#include "Contour.h"
#include "detector.h"

#define KEY_LEFT	0x250000	// 向左
#define KEY_TOP		0x260000	// 向上
#define KEY_RIGHT	0x270000	// 向右
#define KEY_BOTTOM	0x280000	// 向下
#define KEY_DEL		0x2E0000	// 删除

Detector::Detector(cv::Rect rect)
{
    editArea = rect;
    isEditSelectArea = false;
    currentContour.clear();
    eyeglassContours.clear();
}

Detector::~Detector()
{
    eyeglassContours.clear();
    currentContour.clear();
}

std::vector<cv::Point> Detector::findExternalContour(cv::Mat frame, int medianBlurKSize, int morphKSize, cv::Mat background) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    medianBlur(gray, gray, medianBlurKSize);
    blur(gray, gray, cv::Size(3, 3));
    stackBlur(gray, gray, cv::Size(5, 5));
    GaussianBlur(gray, gray, cv::Size(3, 3), 0);

    cv::Mat input = cv::Mat(3, 3, CV_8UC1);
    erode(gray, gray, input);//腐蚀
    dilate(gray, gray, input);//膨胀

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(5, 5));
    clahe->apply(gray, gray);

    Canny(gray, gray, 1, 180, 3);

    cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphKSize, morphKSize));
    cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, morphKernel);// 形态学闭运算

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    //把灰度图缩小后贴到背景右上角展示
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
    erode(gray, gray, input);//腐蚀
    dilate(gray, gray, input);//膨胀

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(gray, gray);

    Canny(gray, gray, 1, 150, 3, true);

    cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphKSize, morphKSize));
    cv::morphologyEx(gray, gray, cv::MORPH_CLOSE, morphKernel);// 形态学闭运算

    // 创建一个与原图像大小相同的掩码图像
    cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);

    // 在掩码图像上绘制并填充给定的轮廓
    cv::drawContours(mask, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);

    // 应用掩码到原图像，只保留轮廓范围内的区域
    gray.setTo(cv::Scalar(0), ~mask);

    // 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(gray, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    //把灰度图缩小后贴到背景右上角展示
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

    fps_.toc();

    char szText[_MAX_PATH] = { 0 };
    sprintf(szText, "%s", fps_.toString().c_str());
    cv::putText(frame, szText, cv::Point(0, frame.rows - 3), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 128, 255));
    
    frame.copyTo(background(cv::Rect(0, 0, frame.cols, frame.rows)));
}

void Detector::drawFrame(cv::Mat frame, cv::Mat background)
{
    fps_.tic();

    if (!eyeglassContours.empty()) cv::drawContours(frame, eyeglassContours, -1, cv::Scalar(0, 255, 0), 2);
    if (!currentContour.empty()) drawContour(frame, currentContour);
    cv::rectangle(frame, selectRect, cv::Scalar(255, 0, 255), 2);

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

void Detector::onKey(int key) {
    switch (key) {
    case KEY_LEFT:
        moveContour(currentContour, selectRect, -1, 0);
        break;

    case KEY_RIGHT:
        moveContour(currentContour, selectRect, 1, 0);
        break;

    case KEY_TOP:
        moveContour(currentContour, selectRect, 0, -1);
        break;

    case KEY_BOTTOM:
        moveContour(currentContour, selectRect, 0, 1);
        break;

    case KEY_DEL:
        deleteContour(currentContour, selectRect);
        break;

    case 'i':
    case 'I':
        insertPoint(currentContour, mousePoint);
        break;
    }
}

void Detector::onMouse(int event, int x, int y) {
    mousePoint = cv::Point(x, y);

    if (event == cv::EVENT_LBUTTONDOWN) {
        isEditSelectArea = true;
        selectRect = cv::Rect(x, y, 0, 0);
    }
    else if (event == cv::EVENT_MOUSEMOVE) {
        if (isEditSelectArea) {
            int rectX = std::min(selectRect.x, x);
            int rectY = std::min(selectRect.y, y);
            int rectWidth = std::abs(x - selectRect.x);
            int rectHeight = std::abs(y - selectRect.y);

            selectRect = cv::Rect(rectX, rectY, rectWidth, rectHeight);
        }
    }
    else if (event == cv::EVENT_LBUTTONUP) {
        isEditSelectArea = false;
        int rectX = std::min(selectRect.x, x);
        int rectY = std::min(selectRect.y, y);
        int rectWidth = std::abs(x - selectRect.x);
        int rectHeight = std::abs(y - selectRect.y);

        selectRect = cv::Rect(rectX, rectY, rectWidth, rectHeight);
    }
}

cv::Rect Detector::getEditArea() {
    return editArea;
}