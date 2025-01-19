///////////////////////////////////////////////////
//				date: 2024.12.28
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include "stdafx.h"
#include "dxf.h"

Dxf::Dxf()
{

}

Dxf::~Dxf()
{

}

bool Dxf::SaveContoursToFile(std::vector<std::vector<cv::Point2f>> contours, char *filename) {
    // 打开DXF文件
    std::ofstream dxfFile(filename);
    if (!dxfFile.is_open()) {
        std::cerr << "Could not open the DXF file" << std::endl;
        return false;
    }

    // 写入DXF文件头
    dxfFile << "  0\nSECTION\n  2\nHEADER\n  0\nENDSEC\n  0\nSECTION\n  2\nENTITIES\n";

    // 写入轮廓
    for (const auto& contour : contours) {
        WriteContourToDxf(dxfFile, contour);
    }

    // 写入DXF文件尾
    dxfFile << "  0\nENDSEC\n  0\nEOF\n";

    // 关闭DXF文件
    dxfFile.close();
    return true;
}

void Dxf::WriteContourToDxf(std::ofstream& dxfFile, const std::vector<cv::Point2f>& contour) {
    dxfFile << "  0\n";
    dxfFile << "POLYLINE\n";
    dxfFile << "  8\n";
    dxfFile << "Contour\n";
    dxfFile << " 66\n";
    dxfFile << "1\n";
    dxfFile << " 10\n";
    dxfFile << "0.0\n";
    dxfFile << " 20\n";
    dxfFile << "0.0\n";
    dxfFile << " 30\n";
    dxfFile << "0.0\n";

    for (const auto& pt : contour) {
        dxfFile << "  0\n";
        dxfFile << "VERTEX\n";
        dxfFile << "  8\n";
        dxfFile << "Contour\n";
        dxfFile << " 10\n";
        dxfFile << pt.x << "\n";
        dxfFile << " 20\n";
        dxfFile << pt.y << "\n";
        dxfFile << " 30\n";
        dxfFile << "0.0\n";
    }

    dxfFile << "  0\n";
    dxfFile << "SEQEND\n";
}