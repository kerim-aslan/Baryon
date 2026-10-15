#pragma once

#include <iostream>
#include <cstdlib>
#include <string>

inline int& getFailedCheckCount() {
    static int count = 0;
    return count;
}

inline int& getTotalCheckCount() {
    static int count = 0;
    return count;
}

inline void recordCheck(bool condition, const char* expr, const std::string& msg, const char* file, int line) {
    getTotalCheckCount()++;
    if (!condition) {
        getFailedCheckCount()++;
        std::cerr << "[FAILED] " << file << ":" << line << " CHECK(" << expr << ")";
        if (!msg.empty()) {
            std::cerr << " - " << msg;
        }
        std::cerr << std::endl;
    }
}

#define CHECK(expr) recordCheck((expr), #expr, "", __FILE__, __LINE__)
#define CHECK_MSG(expr, msg) recordCheck((expr), #expr, (msg), __FILE__, __LINE__)

inline int reportAndExit() {
    int failed = getFailedCheckCount();
    int total = getTotalCheckCount();
    if (failed > 0) {
        std::cerr << "\n>>> TEST FAILED: " << failed << " / " << total << " checks failed. <<<\n" << std::endl;
        return 1;
    }
    std::cout << "\n>>> ALL " << total << " CHECKS PASSED. <<<\n" << std::endl;
    return 0;
}
