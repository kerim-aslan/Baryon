#pragma once

#include <iostream>
#include <string>

namespace Baryon::Test {

inline int g_failureCount = 0;
inline int g_assertionCount = 0;

inline void recordCheck(bool cond, const char* expr, const char* file, int line, const char* msg = nullptr) {
    ++g_assertionCount;
    if (!cond) {
        ++g_failureCount;
        std::cerr << "[CHECK FAILED] " << file << ":" << line << " (" << expr << ")";
        if (msg && *msg) {
            std::cerr << " -> " << msg;
        }
        std::cerr << std::endl;
    }
}

inline int getFailureCount() {
    return g_failureCount;
}

inline int getAssertionCount() {
    return g_assertionCount;
}

inline void resetCounts() {
    g_failureCount = 0;
    g_assertionCount = 0;
}

inline int reportAndExit() {
    if (g_failureCount > 0) {
        std::cerr << "\n[TEST FAILED] Total failures: " << g_failureCount 
                  << " / " << g_assertionCount << " assertions." << std::endl;
        return 1;
    }
    std::cout << "\n[TEST PASSED] All " << g_assertionCount << " assertion(s) passed successfully." << std::endl;
    return 0;
}

} // namespace Baryon::Test

#define CHECK(cond) ::Baryon::Test::recordCheck(static_cast<bool>(cond), #cond, __FILE__, __LINE__)
#define CHECK_MSG(cond, msg) ::Baryon::Test::recordCheck(static_cast<bool>(cond), #cond, __FILE__, __LINE__, (msg))
