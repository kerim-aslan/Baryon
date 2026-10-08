/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#include "bdebugger/DumpRecorder.hpp"
#include <Baryon/collision/CollisionShape.hpp>
#include <ranges>

namespace Baryon::Debugger {

const DumpFrame& DumpRecorder::captureStep(const Simulator& sim, float dt) {
    DumpFrame frame;
    frame.frameIndex = mCurrentFrameIndex++;
    frame.simulationTime = (mSimulationTime += dt);
    frame.timeStep = dt;

    const auto& reg = const_cast<Simulator&>(sim).getRegistry();

    // 1. Capture Entities
    {
        const auto& posePool = reg.getComponentPool<Pose>();
        const auto& entities = posePool.getAllEntities();

        for (auto entity : entities) {
            if (!reg.isAlive(entity) || !reg.hasComponent<Pose>(entity)) continue;

            DumpEntityRecord rec;
            rec.id = entity.id;

            const auto& pose = reg.getComponent<Pose>(entity);
            rec.posX = pose.position.x;
            rec.posY = pose.position.y;
            rec.posZ = pose.position.z;
            rec.rotX = pose.orientation.x;
            rec.rotY = pose.orientation.y;
            rec.rotZ = pose.orientation.z;
            rec.rotW = pose.orientation.w;

            if (reg.hasComponent<Core::Motion>(entity)) {
                const auto& motion = reg.getComponent<Core::Motion>(entity);
                rec.linVelX = motion.linearVelocity.x;
                rec.linVelY = motion.linearVelocity.y;
                rec.linVelZ = motion.linearVelocity.z;
                rec.angVelX = motion.angularVelocity.x;
                rec.angVelY = motion.angularVelocity.y;
                rec.angVelZ = motion.angularVelocity.z;
            }

            if (reg.hasComponent<Core::MassProps>(entity)) {
                rec.mass = reg.getComponent<Core::MassProps>(entity).mass;
            }

            if (reg.hasComponent<Core::BodyState>(entity)) {
                const auto& state = reg.getComponent<Core::BodyState>(entity);
                rec.bodyType = static_cast<uint8_t>(state.type);
                rec.isSleeping = state.isSleeping;
                rec.useCCD = state.useCCD;

                if (state.type == Core::BodyType::Dynamic) {
                    if (state.isSleeping) frame.sleepingBodyCount++;
                    else frame.activeBodyCount++;

                    // Energy & momentum
                    float v2 = rec.linVelX * rec.linVelX + rec.linVelY * rec.linVelY + rec.linVelZ * rec.linVelZ;
                    frame.totalKineticEnergy += 0.5f * rec.mass * v2;
                    frame.totalLinearMomentum += rec.mass * std::sqrt(v2);
                }
            }

            if (reg.hasComponent<ecs::ColliderData>(entity)) {
                const auto& col = reg.getComponent<ecs::ColliderData>(entity);
                rec.isTrigger = col.isTrigger;
                rec.layer = col.layer;
                rec.mask = col.mask;
                std::visit([&rec](const auto& s) {
                    using T = std::decay_t<decltype(s)>;
                    if constexpr (std::is_same_v<T, collision::BoxShape>) {
                        rec.shapeKind = ShapeKind::Box;
                        rec.shapeDimX = s.halfExtents.x;
                        rec.shapeDimY = s.halfExtents.y;
                        rec.shapeDimZ = s.halfExtents.z;
                    } else if constexpr (std::is_same_v<T, collision::SphereShape>) {
                        rec.shapeKind = ShapeKind::Sphere;
                        rec.shapeDimX = s.radius;
                    } else if constexpr (std::is_same_v<T, collision::CapsuleShape>) {
                        rec.shapeKind = ShapeKind::Capsule;
                        rec.shapeDimX = s.radius;
                        rec.shapeDimY = s.height;
                    } else if constexpr (std::is_same_v<T, collision::StaticMeshShape>) {
                        rec.shapeKind = ShapeKind::StaticMesh;
                    } else {
                        rec.shapeKind = ShapeKind::Unknown;
                    }
                }, col.shape.getVariant());
            }

            frame.entities.push_back(rec);
        }
    }

    // 2. Capture Contacts
    const auto& manifolds = sim.getManifolds();
    for (const auto& [key, manifold] : manifolds) {
        for (const auto& pt : manifold.contacts) {
            DumpContactRecord c;
            c.entityA = manifold.entityA.id;
            c.entityB = manifold.entityB.id;
            c.normalX = manifold.normal.x;
            c.normalY = manifold.normal.y;
            c.normalZ = manifold.normal.z;
            c.pointX = pt.worldPosition.x;
            c.pointY = pt.worldPosition.y;
            c.pointZ = pt.worldPosition.z;
            c.penetration = pt.penetration;
            c.normalImpulse = pt.normalImpulse;
            frame.contacts.push_back(c);
        }
    }

    // 3. Evaluate Anomalies
    mAnomalyPredictor.evaluate(frame);

    // 4. Live File Stream
    if (mIsLiveRecordingToFile && mLiveFileStream.is_open()) {
        DumpSerializer::writeFrame(mLiveFileStream, frame);
    }

    // 5. Push to Ring Buffer
    mFrameBuffer.push_back(std::move(frame));
    while (mFrameBuffer.size() > mMaxBufferSize) {
        mFrameBuffer.pop_front();
    }

    return mFrameBuffer.back();
}

bool DumpRecorder::startLiveRecording(const std::string& filePath, float fixedTimeStep) {
    stopLiveRecording();

    mLiveFileStream.open(filePath, std::ios::binary | std::ios::trunc);
    if (!mLiveFileStream.is_open()) return false;

    DumpHeader header;
    header.fixedTimeStep = fixedTimeStep;
    header.totalFrames = 0; // Will be updated on stop
    DumpSerializer::writeHeader(mLiveFileStream, header);

    mLiveFilePath = filePath;
    mIsLiveRecordingToFile = true;
    return true;
}

void DumpRecorder::stopLiveRecording() {
    if (mIsLiveRecordingToFile && mLiveFileStream.is_open()) {
        // Rewrite header with actual frame count
        mLiveFileStream.seekp(0);
        DumpHeader header;
        header.totalFrames = mCurrentFrameIndex;
        DumpSerializer::writeHeader(mLiveFileStream, header);
        mLiveFileStream.close();
    }
    mIsLiveRecordingToFile = false;
    mLiveFilePath.clear();
}

bool DumpRecorder::saveBufferToFile(const std::string& filePath, float fixedTimeStep) const {
    std::ofstream os(filePath, std::ios::binary | std::ios::trunc);
    if (!os.is_open()) return false;

    DumpHeader header;
    header.fixedTimeStep = fixedTimeStep;
    header.totalFrames = mFrameBuffer.size();
    if (!DumpSerializer::writeHeader(os, header)) return false;

    for (const auto& frame : mFrameBuffer) {
        if (!DumpSerializer::writeFrame(os, frame)) return false;
    }
    return true;
}

bool DumpRecorder::exportBufferToJson(const std::string& jsonFilePath) const {
    std::ofstream os(jsonFilePath);
    if (!os.is_open()) return false;

    os << "{\n";
    os << "  \"totalFrames\": " << mFrameBuffer.size() << ",\n";
    os << "  \"frames\": [\n";
    for (size_t i = 0; i < mFrameBuffer.size(); ++i) {
        os << DumpSerializer::frameToJson(mFrameBuffer[i]);
        if (i + 1 < mFrameBuffer.size()) os << ",";
        os << "\n";
    }
    os << "  ]\n";
    os << "}\n";
    return true;
}

} // namespace Baryon::Debugger
