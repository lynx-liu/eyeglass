///////////////////////////////////////////////////
//				date: 2024.12.28
//				author: 刘立向  
//				email: 13651417694@126.com
//				qq: 515311445
///////////////////////////////////////////////////

#include "dxf.h"
#include <codecvt>

Dxf::Dxf()
{

}

Dxf::~Dxf()
{

}

bool Dxf::SaveContoursToFile(std::vector<std::vector<cv::Point2f>> contours, std::string utf8Path, double pixelToMm) {
#ifdef _WIN32
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    std::wstring widePath = converter.from_bytes(utf8Path);

    // 打开DXF文件
    std::ofstream dxfFile(widePath);
#else
    std::ofstream dxfFile(utf8Path);
#endif // _WIN32
    if (!dxfFile.is_open()) {
        std::cerr << "Could not open the DXF file" << std::endl;
        return false;
    }

    // 写入DXF文件头
    dxfFile << "  0\nSECTION\n  2\nHEADER\n  9\n$INSUNITS\n  70\n4\n  0\nENDSEC\n  0\nSECTION\n  2\nENTITIES\n";

    // 写入轮廓
    for (const auto& contour : contours) {
        WriteContourToDxf(dxfFile, contour, pixelToMm);
    }

    // 写入DXF文件尾
    dxfFile << "  0\nENDSEC\n  0\nEOF\n";

    // 关闭DXF文件
    dxfFile.close();
    return true;
}

void Dxf::WriteContourToDxf(std::ofstream& dxfFile, const std::vector<cv::Point2f>& contour, double pixelToMm) {
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
        dxfFile << pt.x * pixelToMm << "\n";
        dxfFile << " 20\n";
        dxfFile << -pt.y * pixelToMm << "\n";
        dxfFile << " 30\n";
        dxfFile << "0.0\n";
    }

    dxfFile << "  0\n";
    dxfFile << "SEQEND\n";
}