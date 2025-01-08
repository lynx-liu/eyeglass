///////////////////////////////////////////////////
//				date: 2024.12.28
//				author: ¡ı¡¢œÚ  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#ifndef DXF_H
#define DXF_H

#include <fstream>

class Dxf {
public:
    Dxf();
    ~Dxf();
    static bool SaveContoursToFile(std::vector<std::vector<cv::Point>> contours, char* filename);

private:
    static void WriteContourToDxf(std::ofstream& dxfFile, const std::vector<cv::Point>& contour);
};

#endif // DXF_H
