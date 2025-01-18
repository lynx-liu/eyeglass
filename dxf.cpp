///////////////////////////////////////////////////
//				date: 2024.12.28
//				author: ������  
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
    // ��DXF�ļ�
    std::ofstream dxfFile(filename);
    if (!dxfFile.is_open()) {
        std::cerr << "Could not open the DXF file" << std::endl;
        return false;
    }

    // д��DXF�ļ�ͷ
    dxfFile << "  0\nSECTION\n  2\nHEADER\n  0\nENDSEC\n  0\nSECTION\n  2\nENTITIES\n";

    // д������
    for (const auto& contour : contours) {
        WriteContourToDxf(dxfFile, contour);
    }

    // д��DXF�ļ�β
    dxfFile << "  0\nENDSEC\n  0\nEOF\n";

    // �ر�DXF�ļ�
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