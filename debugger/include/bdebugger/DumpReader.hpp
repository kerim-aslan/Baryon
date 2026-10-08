/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#pragma once

#include "DumpFormat.hpp"
#include <vector>
#include <string>

namespace Baryon::Debugger {

class DumpReader {
private:
    DumpHeader mHeader;
    std::vector<DumpFrame> mFrames;
    std::string mLoadedFilePath;
    size_t mCurrentFrameIndex{0};

public:
    DumpReader() = default;

    bool loadFromFile(const std::string& filePath);
    void close();

    [[nodiscard]] bool isLoaded() const { return !mFrames.empty(); }
    [[nodiscard]] const std::string& getFilePath() const { return mLoadedFilePath; }
    [[nodiscard]] const DumpHeader& getHeader() const { return mHeader; }

    [[nodiscard]] size_t getFrameCount() const { return mFrames.size(); }
    [[nodiscard]] size_t getCurrentIndex() const { return mCurrentFrameIndex; }
    void setCurrentIndex(size_t index) {
        if (!mFrames.empty()) {
            mCurrentFrameIndex = std::min(index, mFrames.size() - 1);
        }
    }

    void stepForward() {
        if (!mFrames.empty() && mCurrentFrameIndex + 1 < mFrames.size()) {
            mCurrentFrameIndex++;
        }
    }

    void stepBackward() {
        if (!mFrames.empty() && mCurrentFrameIndex > 0) {
            mCurrentFrameIndex--;
        }
    }

    [[nodiscard]] const DumpFrame* getCurrentFrame() const {
        if (mCurrentFrameIndex < mFrames.size()) {
            return &mFrames[mCurrentFrameIndex];
        }
        return nullptr;
    }

    [[nodiscard]] const DumpFrame* getFrameAt(size_t index) const {
        if (index < mFrames.size()) {
            return &mFrames[index];
        }
        return nullptr;
    }

    [[nodiscard]] const std::vector<DumpFrame>& getAllFrames() const { return mFrames; }

    /**
     * @brief Finds frame indices that contain any anomaly.
     */
    [[nodiscard]] std::vector<size_t> getAnomalyFrameIndices() const {
        std::vector<size_t> result;
        for (size_t i = 0; i < mFrames.size(); ++i) {
            if (!mFrames[i].anomalies.empty()) {
                result.push_back(i);
            }
        }
        return result;
    }
};

} // namespace Baryon::Debugger
