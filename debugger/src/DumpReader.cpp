/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#include "bdebugger/DumpReader.hpp"
#include <fstream>

namespace Baryon::Debugger {

bool DumpReader::loadFromFile(const std::string& filePath) {
    close();

    std::ifstream is(filePath, std::ios::binary);
    if (!is.is_open()) return false;

    if (!DumpSerializer::readHeader(is, mHeader)) {
        return false;
    }

    DumpFrame frame;
    while (DumpSerializer::readFrame(is, frame)) {
        mFrames.push_back(std::move(frame));
        frame = DumpFrame{};
    }

    mLoadedFilePath = filePath;
    mCurrentFrameIndex = 0;
    return !mFrames.empty();
}

void DumpReader::close() {
    mFrames.clear();
    mLoadedFilePath.clear();
    mCurrentFrameIndex = 0;
    mHeader = DumpHeader{};
}

} // namespace Baryon::Debugger
