#include "TestCommon.hpp"
#include <Baryon/collision/PhysicsConstants.hpp>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>

using namespace Baryon;
using namespace Baryon::physics;
namespace fs = std::filesystem;

int main() {
    std::cout << "=== V10: Quality Checks, Magic Numbers & const_cast Elimination Test ===" << std::endl;

    // 1. C8: Baryon motor kaynak kodunda (baryon/src ve baryon/include) sifir const_cast kontrolu
    int constCastCount = 0;
    fs::path baryonRoot = SOURCE_ROOT_DIR "/baryon";

    for (const auto& entry : fs::recursive_directory_iterator(baryonRoot)) {
        if (!entry.is_regular_file()) continue;
        auto ext = entry.path().extension().string();
        if (ext != ".hpp" && ext != ".cpp") continue;

        std::ifstream file(entry.path());
        std::string line;
        int lineNum = 0;
        while (std::getline(file, line)) {
            lineNum++;
            if (line.find("const_cast") != std::string::npos) {
                // Ignore comments
                std::string trimmed = line;
                trimmed.erase(0, trimmed.find_first_not_of(" \t"));
                if (trimmed.rfind("//", 0) != 0 && trimmed.rfind("/*", 0) != 0) {
                    std::cout << "  [const_cast found] " << entry.path().filename().string()
                              << ":" << lineNum << " -> " << line << std::endl;
                    constCastCount++;
                }
            }
        }
    }

    std::cout << "  Toplam const_cast sayisi: " << constCastCount << std::endl;
    CHECK_MSG(constCastCount == 0, "Baryon kaynak kodunda sifir const_cast olmali (C8 kurali)");

    // 2. C6: PhysicsConstants icinde merkezi sihirli sayilarin mevcudiyeti
    CHECK_MSG(PhysicsConstants::ManifoldNormalAlignmentThreshold == 0.95f,
              "ManifoldNormalAlignmentThreshold 0.95f olarak tanimli olmali");
    CHECK_MSG(PhysicsConstants::ContactPointMatchRadiusSq == 0.04f,
              "ContactPointMatchRadiusSq 0.04f olarak tanimli olmali");
    CHECK_MSG(PhysicsConstants::AabbVelocityInflationTime == 0.02f,
              "AabbVelocityInflationTime 0.02f olarak tanimli olmali");
    CHECK_MSG(PhysicsConstants::RollingFrictionCoefficient == 0.02f,
              "RollingFrictionCoefficient 0.02f olarak tanimli olmali");
    CHECK_MSG(PhysicsConstants::RollingFrictionAngularVelocityThreshold == 1e-4f,
              "RollingFrictionAngularVelocityThreshold 1e-4f olarak tanimli olmali");
    CHECK_MSG(PhysicsConstants::DefaultMeshQueryExtent == 3.0f,
              "DefaultMeshQueryExtent 3.0f olarak tanimli olmali");
    CHECK_MSG(PhysicsConstants::Infinity == 1e10f,
              "Infinity 1e10f olarak tanimli olmali");

    // 3. C9: [[nodiscard]] donus degerini gozardi etmenin -Werror altinda derleme hatasi vermesi testi
    {
        fs::path failSource = SOURCE_ROOT_DIR "/tests/nodiscard_fail.cpp";
        fs::path includeDir = SOURCE_ROOT_DIR "/baryon/include";
        fs::path outObj = fs::temp_directory_path() / "nodiscard_test.o";
        fs::path outLog = fs::temp_directory_path() / "nodiscard_output.txt";
        std::string cmd = "c++ -std=c++23 -I \"" + includeDir.string() +
                          "\" -Werror=unused-result -c \"" + failSource.string() +
                          "\" -o \"" + outObj.string() + "\" > \"" + outLog.string() + "\" 2>&1";
        int compileResult = std::system(cmd.c_str());
        std::cout << "  [nodiscard test] Komut donus kodu: " << compileResult << std::endl;
        CHECK_MSG(compileResult != 0,
                  "[[nodiscard]] degeri atlandiginda derleyici -Werror ile basarisiz olmali (C9 kurali)");
    }

    // 4. LICENSE dosya adi kontrolu (LICENCE yerine standart LICENSE olmali)
    CHECK_MSG(fs::exists(SOURCE_ROOT_DIR "/LICENSE") && !fs::exists(SOURCE_ROOT_DIR "/LICENCE"),
              "LICENSE dosya adi LICENSE (S ile) olmali");

    return reportAndExit();
}
