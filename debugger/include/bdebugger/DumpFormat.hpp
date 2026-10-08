/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace Baryon::Debugger {

constexpr uint32_t DUMP_MAGIC = 0x4E525942; // "BYRN" in little endian
constexpr uint32_t DUMP_VERSION = 1;

enum class AnomalyType : uint8_t {
    TunnelingRisk = 0,
    NumericalInstability = 1,
    ExcessivePenetration = 2,
    SolverJitter = 3,
    ConstraintStrain = 4,
    SleepingAnomaly = 5
};

enum class AnomalySeverity : uint8_t {
    Info = 0,
    Warning = 1,
    Critical = 2
};

struct AnomalyRecord {
    uint64_t frameIndex{0};
    uint32_t entityId{0xFFFFFFFF};
    AnomalyType type{AnomalyType::TunnelingRisk};
    AnomalySeverity severity{AnomalySeverity::Warning};
    std::string description;

    static const char* typeToString(AnomalyType t) {
        switch (t) {
            case AnomalyType::TunnelingRisk: return "Tunelleme Riski";
            case AnomalyType::NumericalInstability: return "Sayisal Kararsizlik (NaN/Inf)";
            case AnomalyType::ExcessivePenetration: return "Asiri Penetrasyon (Ic Ice Gecme)";
            case AnomalyType::SolverJitter: return "Cozucu Titremesi (Jitter)";
            case AnomalyType::ConstraintStrain: return "Kisit Gerilmesi / Kopma";
            case AnomalyType::SleepingAnomaly: return "Uyku Anomalisi";
            default: return "Bilinmeyen";
        }
    }

    static const char* severityToString(AnomalySeverity s) {
        switch (s) {
            case AnomalySeverity::Info: return "BILGI";
            case AnomalySeverity::Warning: return "UYARI";
            case AnomalySeverity::Critical: return "KRITIK";
            default: return "BILINMEYEN";
        }
    }
};

enum class ShapeKind : uint8_t {
    Box = 0,
    Sphere = 1,
    Capsule = 2,
    StaticMesh = 3,
    Unknown = 255
};

struct DumpEntityRecord {
    uint32_t id{0};
    uint8_t bodyType{2}; // 0: Static, 1: Kinematic, 2: Dynamic
    bool isSleeping{false};
    bool useCCD{true};
    bool isTrigger{false};
    uint32_t layer{0x0001};
    uint32_t mask{0xFFFF};
    float mass{1.0f};

    // Position & Orientation
    float posX{0.0f}, posY{0.0f}, posZ{0.0f};
    float rotX{0.0f}, rotY{0.0f}, rotZ{0.0f}, rotW{1.0f};

    // Velocities
    float linVelX{0.0f}, linVelY{0.0f}, linVelZ{0.0f};
    float angVelX{0.0f}, angVelY{0.0f}, angVelZ{0.0f};

    // Shape Geometry
    ShapeKind shapeKind{ShapeKind::Box};
    float shapeDimX{1.0f}, shapeDimY{1.0f}, shapeDimZ{1.0f}; // Box halfExtents, or radius/height
    float shapeDimW{0.0f};
};

struct DumpContactRecord {
    uint32_t entityA{0};
    uint32_t entityB{0};
    float normalX{0.0f}, normalY{0.0f}, normalZ{0.0f};
    float pointX{0.0f}, pointY{0.0f}, pointZ{0.0f};
    float penetration{0.0f};
    float normalImpulse{0.0f};
};

struct DumpFrame {
    uint64_t frameIndex{0};
    float simulationTime{0.0f};
    float timeStep{1.0f / 60.0f};

    // Telemetry aggregates
    float totalKineticEnergy{0.0f};
    float totalLinearMomentum{0.0f};
    uint32_t activeBodyCount{0};
    uint32_t sleepingBodyCount{0};

    std::vector<DumpEntityRecord> entities;
    std::vector<DumpContactRecord> contacts;
    std::vector<AnomalyRecord> anomalies;
};

struct DumpHeader {
    uint32_t magic{DUMP_MAGIC};
    uint32_t version{DUMP_VERSION};
    uint64_t totalFrames{0};
    float fixedTimeStep{1.0f / 60.0f};
};

// Serialization and Deserialization Utilities
class DumpSerializer {
public:
    static bool writeHeader(std::ostream& os, const DumpHeader& header) {
        os.write(reinterpret_cast<const char*>(&header), sizeof(DumpHeader));
        return os.good();
    }

    static bool readHeader(std::istream& is, DumpHeader& header) {
        is.read(reinterpret_cast<char*>(&header), sizeof(DumpHeader));
        if (!is.good()) return false;
        return (header.magic == DUMP_MAGIC && header.version == DUMP_VERSION);
    }

    static bool writeFrame(std::ostream& os, const DumpFrame& frame) {
        os.write(reinterpret_cast<const char*>(&frame.frameIndex), sizeof(frame.frameIndex));
        os.write(reinterpret_cast<const char*>(&frame.simulationTime), sizeof(frame.simulationTime));
        os.write(reinterpret_cast<const char*>(&frame.timeStep), sizeof(frame.timeStep));
        os.write(reinterpret_cast<const char*>(&frame.totalKineticEnergy), sizeof(frame.totalKineticEnergy));
        os.write(reinterpret_cast<const char*>(&frame.totalLinearMomentum), sizeof(frame.totalLinearMomentum));
        os.write(reinterpret_cast<const char*>(&frame.activeBodyCount), sizeof(frame.activeBodyCount));
        os.write(reinterpret_cast<const char*>(&frame.sleepingBodyCount), sizeof(frame.sleepingBodyCount));

        // Entities
        uint32_t entityCount = static_cast<uint32_t>(frame.entities.size());
        os.write(reinterpret_cast<const char*>(&entityCount), sizeof(entityCount));
        if (entityCount > 0) {
            os.write(reinterpret_cast<const char*>(frame.entities.data()), entityCount * sizeof(DumpEntityRecord));
        }

        // Contacts
        uint32_t contactCount = static_cast<uint32_t>(frame.contacts.size());
        os.write(reinterpret_cast<const char*>(&contactCount), sizeof(contactCount));
        if (contactCount > 0) {
            os.write(reinterpret_cast<const char*>(frame.contacts.data()), contactCount * sizeof(DumpContactRecord));
        }

        // Anomalies
        uint32_t anomalyCount = static_cast<uint32_t>(frame.anomalies.size());
        os.write(reinterpret_cast<const char*>(&anomalyCount), sizeof(anomalyCount));
        for (const auto& a : frame.anomalies) {
            os.write(reinterpret_cast<const char*>(&a.frameIndex), sizeof(a.frameIndex));
            os.write(reinterpret_cast<const char*>(&a.entityId), sizeof(a.entityId));
            os.write(reinterpret_cast<const char*>(&a.type), sizeof(a.type));
            os.write(reinterpret_cast<const char*>(&a.severity), sizeof(a.severity));
            uint32_t strLen = static_cast<uint32_t>(a.description.size());
            os.write(reinterpret_cast<const char*>(&strLen), sizeof(strLen));
            if (strLen > 0) {
                os.write(a.description.data(), strLen);
            }
        }

        return os.good();
    }

    static bool readFrame(std::istream& is, DumpFrame& frame) {
        if (!is.read(reinterpret_cast<char*>(&frame.frameIndex), sizeof(frame.frameIndex))) return false;
        is.read(reinterpret_cast<char*>(&frame.simulationTime), sizeof(frame.simulationTime));
        is.read(reinterpret_cast<char*>(&frame.timeStep), sizeof(frame.timeStep));
        is.read(reinterpret_cast<char*>(&frame.totalKineticEnergy), sizeof(frame.totalKineticEnergy));
        is.read(reinterpret_cast<char*>(&frame.totalLinearMomentum), sizeof(frame.totalLinearMomentum));
        is.read(reinterpret_cast<char*>(&frame.activeBodyCount), sizeof(frame.activeBodyCount));
        is.read(reinterpret_cast<char*>(&frame.sleepingBodyCount), sizeof(frame.sleepingBodyCount));

        // Entities
        uint32_t entityCount = 0;
        is.read(reinterpret_cast<char*>(&entityCount), sizeof(entityCount));
        frame.entities.resize(entityCount);
        if (entityCount > 0) {
            is.read(reinterpret_cast<char*>(frame.entities.data()), entityCount * sizeof(DumpEntityRecord));
        }

        // Contacts
        uint32_t contactCount = 0;
        is.read(reinterpret_cast<char*>(&contactCount), sizeof(contactCount));
        frame.contacts.resize(contactCount);
        if (contactCount > 0) {
            is.read(reinterpret_cast<char*>(frame.contacts.data()), contactCount * sizeof(DumpContactRecord));
        }

        // Anomalies
        uint32_t anomalyCount = 0;
        is.read(reinterpret_cast<char*>(&anomalyCount), sizeof(anomalyCount));
        frame.anomalies.resize(anomalyCount);
        for (uint32_t i = 0; i < anomalyCount; ++i) {
            auto& a = frame.anomalies[i];
            is.read(reinterpret_cast<char*>(&a.frameIndex), sizeof(a.frameIndex));
            is.read(reinterpret_cast<char*>(&a.entityId), sizeof(a.entityId));
            is.read(reinterpret_cast<char*>(&a.type), sizeof(a.type));
            is.read(reinterpret_cast<char*>(&a.severity), sizeof(a.severity));
            uint32_t strLen = 0;
            is.read(reinterpret_cast<char*>(&strLen), sizeof(strLen));
            a.description.resize(strLen);
            if (strLen > 0) {
                is.read(&a.description[0], strLen);
            }
        }

        return is.good();
    }

    static std::string frameToJson(const DumpFrame& frame) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(5);
        ss << "{\n";
        ss << "  \"frameIndex\": " << frame.frameIndex << ",\n";
        ss << "  \"simulationTime\": " << frame.simulationTime << ",\n";
        ss << "  \"timeStep\": " << frame.timeStep << ",\n";
        ss << "  \"totalKineticEnergy\": " << frame.totalKineticEnergy << ",\n";
        ss << "  \"totalLinearMomentum\": " << frame.totalLinearMomentum << ",\n";
        ss << "  \"activeBodies\": " << frame.activeBodyCount << ",\n";
        ss << "  \"sleepingBodies\": " << frame.sleepingBodyCount << ",\n";

        // Entities
        ss << "  \"entities\": [\n";
        for (size_t i = 0; i < frame.entities.size(); ++i) {
            const auto& e = frame.entities[i];
            ss << "    {\"id\": " << e.id << ", \"type\": " << (int)e.bodyType 
               << ", \"pos\": [" << e.posX << ", " << e.posY << ", " << e.posZ << "]"
               << ", \"vel\": [" << e.linVelX << ", " << e.linVelY << ", " << e.linVelZ << "]"
               << ", \"mass\": " << e.mass << ", \"sleeping\": " << (e.isSleeping ? "true" : "false") << "}";
            if (i + 1 < frame.entities.size()) ss << ",";
            ss << "\n";
        }
        ss << "  ],\n";

        // Contacts
        ss << "  \"contacts\": [\n";
        for (size_t i = 0; i < frame.contacts.size(); ++i) {
            const auto& c = frame.contacts[i];
            ss << "    {\"entityA\": " << c.entityA << ", \"entityB\": " << c.entityB
               << ", \"point\": [" << c.pointX << ", " << c.pointY << ", " << c.pointZ << "]"
               << ", \"normal\": [" << c.normalX << ", " << c.normalY << ", " << c.normalZ << "]"
               << ", \"penetration\": " << c.penetration << ", \"normalImpulse\": " << c.normalImpulse << "}";
            if (i + 1 < frame.contacts.size()) ss << ",";
            ss << "\n";
        }
        ss << "  ],\n";

        // Anomalies
        ss << "  \"anomalies\": [\n";
        for (size_t i = 0; i < frame.anomalies.size(); ++i) {
            const auto& a = frame.anomalies[i];
            ss << "    {\"severity\": \"" << AnomalyRecord::severityToString(a.severity) << "\""
               << ", \"type\": \"" << AnomalyRecord::typeToString(a.type) << "\""
               << ", \"entityId\": " << a.entityId
               << ", \"description\": \"" << a.description << "\"}";
            if (i + 1 < frame.anomalies.size()) ss << ",";
            ss << "\n";
        }
        ss << "  ]\n";
        ss << "}";
        return ss.str();
    }
};

} // namespace Baryon::Debugger
