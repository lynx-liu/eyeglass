///////////////////////////////////////////////////
//				date: 2022.06.30
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#ifndef DETECTOR_H
#define DETECTOR_H

#include <vector>
#include <thread>
#include <opencv2/opencv.hpp>

#define KEY_UNPRESS -1			// 无按键
#define KEY_ESCAPE	27			// ESC 键
#define KEY_SPACE	32			// 空格键
#define KEY_RETURN	13			// 回车键

class Detector {
public:
    Detector();
    ~Detector();
    void detect(cv::Mat frame, double clipLimit, int medianBlurKSize, int morphKSize, cv::Mat background);
    void drawFrame(cv::Mat frame, cv::Mat background);
    void findNext();
    bool saveToDxf(char *filename);
    bool onKey(int key);
    bool onMouse(int event, int x, int y);
    cv::Rect getEditArea();
    void reset(cv::Rect rect = {});

protected:
    std::vector<cv::Point2f> findExternalContour(cv::Mat frame, double clipLimit, int medianBlurKSize, int morphKSize, cv::Mat background);
    std::vector<cv::Point2f> findContourInMask(cv::Mat frame, double clipLimit, int medianBlurKSize, int morphKSize, const std::vector<cv::Point>& contour, cv::Mat background);

private:
    cv::Rect editArea;
    cv::Rect selectRect;
    bool isEditSelectArea;
    cv::Point mousePoint;
    std::vector<cv::Point2f> currentContour;
    std::vector<std::vector<cv::Point2f>> eyeglassContours;
};

#endif // DETECTOR_H









