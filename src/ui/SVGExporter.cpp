#include "SVGExporter.hpp"
#include <windows.h>
#include <commdlg.h>

namespace CircuitSim {
namespace FileDialogs {

std::string saveSVGFileDialog(const std::string& title, const std::string& defaultName) {
    char szFile[260] = {0};
    strncpy_s(szFile, defaultName.c_str(), sizeof(szFile) - 1);
    OPENFILENAMEA ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "SVG Vector Graphic (*.svg)\0*.svg\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile);
    ofn.lpstrTitle = title.c_str();
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;
    if (GetSaveFileNameA(&ofn)) {
        std::string res(szFile);
        if (res.find(".svg") == std::string::npos && res.find(".SVG") == std::string::npos) {
            res += ".svg";
        }
        return res;
    }
    return "";
}

std::string saveHTMLFileDialog(const std::string& title, const std::string& defaultName) {
    char szFile[260] = {0};
    strncpy_s(szFile, defaultName.c_str(), sizeof(szFile) - 1);
    OPENFILENAMEA ofn = {0};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "HTML Report (*.html)\0*.html\0All Files (*.*)\0*.*\0";
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile);
    ofn.lpstrTitle = title.c_str();
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;
    if (GetSaveFileNameA(&ofn)) {
        std::string res(szFile);
        if (res.find(".html") == std::string::npos && res.find(".HTML") == std::string::npos) {
            res += ".html";
        }
        return res;
    }
    return "";
}

} // namespace FileDialogs
} // namespace CircuitSim
