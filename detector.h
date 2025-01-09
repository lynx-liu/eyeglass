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
#include <future>

#include "fps.hpp"

class Detector {
public:
    Detector();
    ~Detector();
    void detect(cv::Mat frame, int medianBlurKSize, int morphKSize, cv::Mat background);
    void drawFrame(cv::Mat frame, cv::Mat background);
    void findNext();
    void saveToDxf(char *filename);
    void moveContour(cv::Rect& area, int dx, int dy);
    void deleteContour(cv::Rect& area);

protected:
    std::vector<cv::Point> findExternalContour(cv::Mat frame, int medianBlurKSize, int morphKSize, cv::Mat background);
    std::vector<cv::Point> findContourInMask(cv::Mat frame, int medianBlurKSize, int morphKSize, const std::vector<cv::Point>& contour, cv::Mat background);

private:
    std::vector<cv::Point> currentContour;
    std::vector<std::vector<cv::Point>> eyeglassContours;
    static std::vector<cv::Point> scaleContour(const std::vector<cv::Point>& contour, int N);
    static int findMaxContourId(std::vector<std::vector<cv::Point> > contours);
    static void drawContour(cv::Mat background, std::vector<cv::Point> contour);
    FPS fps_ = FPS();
};

#endif // DETECTOR_H









