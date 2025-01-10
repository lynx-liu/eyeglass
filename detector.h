///////////////////////////////////////////////////
//				date: 2022.06.30
//				author: ¡ı¡¢œÚ  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#ifndef DETECTOR_H
#define DETECTOR_H

#include <vector>
#include <thread>

#include "fps.hpp"

class Detector {
public:
    Detector(cv::Rect rect);
    ~Detector();
    void detect(cv::Mat frame, int medianBlurKSize, int morphKSize, cv::Mat background);
    void drawFrame(cv::Mat frame, cv::Mat background);
    void findNext();
    void saveToDxf(char *filename);
    void onKey(int key);
    void onMouse(int event, int x, int y);
    cv::Rect getEditArea();

protected:
    std::vector<cv::Point> findExternalContour(cv::Mat frame, int medianBlurKSize, int morphKSize, cv::Mat background);
    std::vector<cv::Point> findContourInMask(cv::Mat frame, int medianBlurKSize, int morphKSize, const std::vector<cv::Point>& contour, cv::Mat background);

private:
    cv::Rect editArea;
    cv::Rect selectRect;
    bool isEditSelectArea;
    cv::Point mousePoint;
    std::vector<cv::Point> currentContour;
    std::vector<std::vector<cv::Point>> eyeglassContours;
    FPS fps_ = FPS();
};

#endif // DETECTOR_H









