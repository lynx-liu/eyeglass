///////////////////////////////////////////////////
//				date: 2025.01.10
//				author: ¡ı¡¢œÚ  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#ifndef CONTOUR_H
#define CONTOUR_H

std::vector<cv::Point> scaleContour(const std::vector<cv::Point>& contour, int N);
int findMaxContourId(const std::vector<std::vector<cv::Point> >& contours);
void drawContour(cv::Mat background, const std::vector<cv::Point>& contour);
void moveContour(std::vector<cv::Point>& contour, cv::Rect& area, int dx, int dy);
void deleteContour(std::vector<cv::Point>& contour, cv::Rect& area);
double distance(const cv::Point& pt1, const cv::Point& pt2);
bool isPointBetween(const cv::Point& p1, const cv::Point& p2, const cv::Point& newPoint);
void insertPoint(std::vector<cv::Point>& contour, const cv::Point& newPoint);
std::vector<cv::Point> smoothContourWithSlidingWindow(const std::vector<cv::Point>& contour, int windowSize = 5);
std::vector<cv::Point> gaussianSmooth(const std::vector<cv::Point>& contour, int kernelSize = 5, double sigma = 1.0);
std::vector<cv::Point> smoothContourWithBezier(const std::vector<cv::Point>& contour, int numPoints = 250, int numThreads = 4);

#endif //CONTOUR_H