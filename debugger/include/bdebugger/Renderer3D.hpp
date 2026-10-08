/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#pragma once

#include "GLFunctions.hpp"
#include <Baryon/math/Vector3.hpp>
#include <Baryon/math/Quaternion.hpp>
#include <Baryon/math/Pose.hpp>
#include <vector>

namespace Baryon::Debugger {

struct Vertex3D {
    float x, y, z;
    float r, g, b, a;
};

class Camera3D {
public:
    Vector3 target{0.0f, 1.5f, 0.0f};
    float distance{18.0f};
    float yaw{45.0f};   // degrees
    float pitch{25.0f}; // degrees

    void orbit(float deltaYaw, float deltaPitch);
    void zoom(float deltaDistance);
    void pan(float deltaX, float deltaY);

    void getMatrices(int viewportWidth, int viewportHeight, float* outMVP) const;
};

class Renderer3D {
private:
    Camera3D mCamera;

    // FBO
    GLuint mFBO{0};
    GLuint mColorTex{0};
    GLuint mDepthRBO{0};
    int mWidth{1280};
    int mHeight{720};

    // Shader & Buffers
    GLuint mProgram{0};
    GLuint mVAO{0};
    GLuint mVBO{0};
    GLint mUniformMVP{-1};

    std::vector<Vertex3D> mLineVertices;

public:
    Renderer3D() = default;
    ~Renderer3D();

    bool init(int width, int height);
    void resize(int width, int height);

    Camera3D& getCamera() { return mCamera; }
    [[nodiscard]] const Camera3D& getCamera() const { return mCamera; }

    [[nodiscard]] GLuint getTextureId() const { return mColorTex; }
    [[nodiscard]] int getWidth() const { return mWidth; }
    [[nodiscard]] int getHeight() const { return mHeight; }

    void beginFrame();
    void endFrame();

    // Drawing helpers
    void addLine(const Vector3& p0, const Vector3& p1, const Vector3& color, float alpha = 1.0f);
    void drawGrid(float size = 30.0f, float step = 1.0f);
    void drawAxes(float length = 2.0f);

    void drawWireBox(const Pose& pose, const Vector3& halfExtents, const Vector3& color);
    void drawWireSphere(const Pose& pose, float radius, const Vector3& color, int segments = 16);
    void drawWireCapsule(const Pose& pose, float radius, float height, const Vector3& color, int segments = 12);
    void drawWireMesh(const Pose& pose, const Vector3& color);
    void drawRaycast(const Vector3& origin, const Vector3& end, bool hasHit, const Vector3& hitPoint, const Vector3& hitNormal);
    void drawContactPoint(const Vector3& pt, const Vector3& normal, float penetration, float normalImpulse);
    void drawVelocityVector(const Vector3& origin, const Vector3& velocity, const Vector3& color);

private:
    bool createFBO();
    void destroyFBO();
    bool createShaders();
};

} // namespace Baryon::Debugger
