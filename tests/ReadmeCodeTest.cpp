#include "TestCommon.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

#ifndef CXX_COMPILER
#define CXX_COMPILER "g++"
#endif

#ifndef BARYON_SOURCE_DIR
#define BARYON_SOURCE_DIR "."
#endif

// Markdown dosyasındaki ilk ```cpp kod bloğunu ayıklar
std::string extractCppBlock(const fs::path& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Dosya acilamadi: " << filePath << std::endl;
        return "";
    }

    std::string line;
    bool inCpp = false;
    std::ostringstream code;

    while (std::getline(file, line)) {
        if (!inCpp) {
            if (line.find("```cpp") != std::string::npos) {
                inCpp = true;
            }
        } else {
            if (line.find("```") != std::string::npos) {
                break;
            }
            code << line << "\n";
        }
    }

    return code.str();
}

int compileSnippet(const std::string& snippet, const std::string& filename) {
    std::ofstream out(filename);
    out << snippet;
    out.close();

    fs::path includePath = fs::path(BARYON_SOURCE_DIR) / "baryon" / "include";

#ifdef _WIN32
    std::string compiler = "g++";
    std::string cmd = compiler + " -std=c++23 -fsyntax-only -I\"" +
                      includePath.string() + "\" \"" + filename + "\"";
#else
    std::string cmd = std::string("\"") + CXX_COMPILER + "\" -std=c++23 -fsyntax-only -I\"" +
                      includePath.string() + "\" \"" + filename + "\"";
#endif

    std::cout << "Executing compile check: " << cmd << std::endl;
    int ret = std::system(cmd.c_str());
    return ret;
}

int main() {
    std::cout << "=== Baryon N4 README Code Snippet Verification ===" << std::endl;

    fs::path baryonReadme = fs::path(BARYON_SOURCE_DIR) / "baryon" / "README.md";
    std::string code1 = extractCppBlock(baryonReadme);
    CHECK_MSG(!code1.empty(), "baryon/README.md cpp blogu bulunamadi");

    int ret1 = compileSnippet(code1, "baryon_readme_snippet.cpp");
    std::cout << "baryon/README.md compilation return code: " << ret1 << std::endl;
    CHECK_MSG(ret1 == 0, "baryon/README.md icindeki ornek kod basariyla derlenmeli");

    fs::path rootReadme = fs::path(BARYON_SOURCE_DIR) / "Readme.md";
    if (fs::exists(rootReadme)) {
        std::string code2 = extractCppBlock(rootReadme);
        if (!code2.empty()) {
            int ret2 = compileSnippet(code2, "root_readme_snippet.cpp");
            std::cout << "Readme.md compilation return code: " << ret2 << std::endl;
            CHECK_MSG(ret2 == 0, "Readme.md icindeki ornek kod basariyla derlenmeli");
        }
    }

    return reportAndExit();
}
