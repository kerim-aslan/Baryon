/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#include "bdebugger/DebuggerApp.hpp"
#include "bdebugger/DumpReader.hpp"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    std::cout << "====================================================\n";
    std::cout << "  Baryon Physics Diagnostic Tool & Visual Debugger  \n";
    std::cout << "  (C) 2026 Kerim Aslan | Pure C++23 Physics Engine  \n";
    std::cout << "====================================================\n";

    // CLI mode: offline analysis of dump files without opening window if requested
    if (argc >= 3 && std::string(argv[1]) == "--analyze") {
        std::string dumpFile = argv[2];
        std::cout << "[CLI] Analyzing dump file: " << dumpFile << "...\n";
        Baryon::Debugger::DumpReader reader;
        if (!reader.loadFromFile(dumpFile)) {
            std::cerr << "[CLI ERROR] Failed to open dump file: " << dumpFile << "\n";
            return 1;
        }

        std::cout << "[CLI] Total Frames: " << reader.getFrameCount() << "\n";
        auto anomalyIndices = reader.getAnomalyFrameIndices();
        std::cout << "[CLI] Anomalous Frames Found: " << anomalyIndices.size() << "\n";
        for (size_t fIdx : anomalyIndices) {
            const auto* f = reader.getFrameAt(fIdx);
            if (f) {
                std::cout << "  -> Frame " << f->frameIndex << " (t=" << f->simulationTime << "s): "
                          << f->anomalies.size() << " alert(s)\n";
                for (const auto& a : f->anomalies) {
                    std::cout << "     [" << Baryon::Debugger::AnomalyRecord::severityToString(a.severity)
                              << "] " << Baryon::Debugger::AnomalyRecord::typeToString(a.type)
                              << ": " << a.description << "\n";
                }
            }
        }
        return 0;
    }

    // Interactive GUI mode
    Baryon::Debugger::DebuggerApp app;
    if (!app.init()) {
        std::cerr << "[FATAL] Failed to initialize DebuggerApp!\n";
        return 1;
    }

    app.run();
    return 0;
}
