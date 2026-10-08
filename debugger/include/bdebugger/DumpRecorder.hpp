/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#pragma once

#include "DumpFormat.hpp"
#include "AnomalyPredictor.hpp"
#include <Baryon/Simulator.hpp>

#include <deque>
#include <string>
#include <fstream>
#include <memory>

namespace Baryon::Debugger {

class DumpRecorder {
private:
    size_t mMaxBufferSize{1200}; // Holds 20 seconds at 60 FPS
    std::deque<DumpFrame> mFrameBuffer;
    AnomalyPredictor mAnomalyPredictor;

    uint64_t mCurrentFrameIndex{0};
    float mSimulationTime{0.0f};

    bool mIsLiveRecordingToFile{false};
    std::ofstream mLiveFileStream;
    std::string mLiveFilePath;

public:
    DumpRecorder() = default;
    explicit DumpRecorder(size_t maxBufferSize) : mMaxBufferSize(maxBufferSize) {}
    ~DumpRecorder() { stopLiveRecording(); }

    void setBufferSize(size_t size) { mMaxBufferSize = size; }
    [[nodiscard]] size_t getBufferSize() const { return mMaxBufferSize; }

    AnomalyPredictor& getAnomalyPredictor() { return mAnomalyPredictor; }
    const AnomalyPredictor& getAnomalyPredictor() const { return mAnomalyPredictor; }

    void reset() {
        mFrameBuffer.clear();
        mAnomalyPredictor.reset();
        mCurrentFrameIndex = 0;
        mSimulationTime = 0.0f;
    }

    /**
     * @brief Extracts current physics state from Simulator, evaluates anomalies,
     *        pushes to ring buffer, and streams to file if live recording is active.
     */
    const DumpFrame& captureStep(const Simulator& sim, float dt);

    /**
     * @brief Starts live streaming recording directly to a .baryon_dump file.
     */
    bool startLiveRecording(const std::string& filePath, float fixedTimeStep);

    /**
     * @brief Stops live recording and flushes the file header with total frames.
     */
    void stopLiveRecording();

    [[nodiscard]] bool isLiveRecording() const { return mIsLiveRecordingToFile; }

    /**
     * @brief Dumps current buffered frames to a .baryon_dump file (Blackbox export).
     */
    bool saveBufferToFile(const std::string& filePath, float fixedTimeStep) const;

    /**
     * @brief Exports buffered frames to a human-readable JSON file.
     */
    bool exportBufferToJson(const std::string& jsonFilePath) const;

    [[nodiscard]] const std::deque<DumpFrame>& getBuffer() const { return mFrameBuffer; }
    [[nodiscard]] size_t getRecordedFrameCount() const { return mFrameBuffer.size(); }
    [[nodiscard]] const DumpFrame* getLatestFrame() const {
        return mFrameBuffer.empty() ? nullptr : &mFrameBuffer.back();
    }
    [[nodiscard]] const DumpFrame* getFrameAtBufferIndex(size_t idx) const {
        if (idx >= mFrameBuffer.size()) return nullptr;
        return &mFrameBuffer[idx];
    }
};

} // namespace Baryon::Debugger
