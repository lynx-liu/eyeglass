///////////////////////////////////////////////////
//				date: 2024.12.28
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#ifndef DXF_H
#define DXF_H

#include <fstream>
#include <opencv2/opencv.hpp>
#define CV_VERSION_ID CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) \
	CVAUX_STR(CV_SUBMINOR_VERSION)

#ifdef _DEBUG
#pragma comment(lib,"opencv_world" CV_VERSION_ID "d.lib")
#else
#pragma comment(lib,"opencv_world" CV_VERSION_ID ".lib")
#endif

class Dxf {
public:
    Dxf();
    ~Dxf();
    static bool SaveContoursToFile(std::vector<std::vector<cv::Point2f>> contours, std::string utf8Path);

private:
    static void WriteContourToDxf(std::ofstream& dxfFile, const std::vector<cv::Point2f>& contour);
};

#endif // DXF_H
