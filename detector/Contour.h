///////////////////////////////////////////////////
//				date: 2025.01.10
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#ifndef CONTOUR_H
#define CONTOUR_H

#include <opencv2/opencv.hpp>

inline double distance(const cv::Point2f& pt1, const cv::Point2f& pt2);

std::vector<cv::Point2f> convertToPoint2f(const std::vector<cv::Point>& contour);
std::vector<cv::Point> scaleContour(const std::vector<cv::Point2f>& contour, int N);
int findMaxContourId(const std::vector<std::vector<cv::Point> >& contours);
void drawContour(cv::Mat background, const std::vector<cv::Point2f>& contour, cv::Scalar scalar = cv::Scalar(255, 0, 0), bool markPoint = false);
void drawContours(cv::Mat background, const std::vector<std::vector<cv::Point2f> >& contours, cv::Scalar scalar);
void moveContour(std::vector<cv::Point2f>& contour, cv::Rect& area, int dx, int dy);
void deleteContour(std::vector<cv::Point2f>& contour, cv::Rect& area);
bool isPointBetween(const cv::Point2f& p1, const cv::Point2f& p2, const cv::Point2f& newPoint);
void insertPoint(std::vector<cv::Point2f>& contour, const cv::Point& newPoint);
std::vector<cv::Point2f> smoothContourWithSlidingWindow(const std::vector<cv::Point2f>& contour, int windowSize = 15);
std::vector<cv::Point2f> gaussianSmooth(const std::vector<cv::Point>& contour, int kernelSize = 0, double sigma = 5.0);
std::vector<cv::Point2f> gaussianSmooth(const std::vector<cv::Point2f>& contour, int kernelSize = 0, double sigma = 5.0);
std::vector<cv::Point2f> smoothContourWithBezier(const std::vector<cv::Point2f>& contour);
std::vector<cv::Point2f> smoothContourWithBilateral(const std::vector<cv::Point2f>& contour, int windowSize = 15, double spatialSigma = 10.0, double intensitySigma = 50.0);

#endif //CONTOUR_H