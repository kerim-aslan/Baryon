/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#include "bdebugger/Renderer3D.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace Baryon::Debugger {

static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;

void Camera3D::orbit(float deltaYaw, float deltaPitch) {
    yaw += deltaYaw;
    pitch += deltaPitch;
    pitch = std::clamp(pitch, -89.0f, 89.0f);
}

void Camera3D::zoom(float deltaDistance) {
    distance -= deltaDistance;
    if (distance < 1.0f) distance = 1.0f;
    if (distance > 200.0f) distance = 200.0f;
}

void Camera3D::pan(float deltaX, float deltaY) {
    float radYaw = yaw * DEG_TO_RAD;
    Vector3 right(std::cos(radYaw), 0.0f, -std::sin(radYaw));
    Vector3 up(0.0f, 1.0f, 0.0f);
    target = target - right * deltaX + up * deltaY;
}

void Camera3D::getMatrices(int viewportWidth, int viewportHeight, float* outMVP) const {
    float radYaw = yaw * DEG_TO_RAD;
    float radPitch = pitch * DEG_TO_RAD;

    // Eye position
    float cx = distance * std::cos(radPitch) * std::sin(radYaw);
    float cy = distance * std::sin(radPitch);
    float cz = distance * std::cos(radPitch) * std::cos(radYaw);
    Vector3 eye = target + Vector3(cx, cy, cz);

    // LookAt matrix
    Vector3 f = (target - eye).getNormalized();
    Vector3 up(0.0f, 1.0f, 0.0f);
    Vector3 s = f.cross(up).getNormalized();
    Vector3 u = s.cross(f);

    float V[16] = {
        s.x,  u.x, -f.x, 0.0f,
        s.y,  u.y, -f.y, 0.0f,
        s.z,  u.z, -f.z, 0.0f,
       -s.dot(eye), -u.dot(eye), f.dot(eye), 1.0f
    };

    // Perspective matrix
    float aspect = (viewportHeight > 0) ? (static_cast<float>(viewportWidth) / static_cast<float>(viewportHeight)) : 1.0f;
    float fov = 45.0f * DEG_TO_RAD;
    float tanHalfFov = std::tan(fov * 0.5f);
    float nearPlane = 0.1f;
    float farPlane = 500.0f;

    float P[16] = {0};
    P[0] = 1.0f / (aspect * tanHalfFov);
    P[5] = 1.0f / tanHalfFov;
    P[10] = -(farPlane + nearPlane) / (farPlane - nearPlane);
    P[11] = -1.0f;
    P[14] = -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane);

    // Multiply P * V (column-major)
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += P[k * 4 + j] * V[i * 4 + k];
            }
            outMVP[i * 4 + j] = sum;
        }
    }
}

Renderer3D::~Renderer3D() {
    destroyFBO();
    if (mVAO) { GL::glDeleteVertexArrays(1, &mVAO); mVAO = 0; }
    if (mVBO) { GL::glDeleteBuffers(1, &mVBO); mVBO = 0; }
    if (mProgram) { GL::glDeleteProgram(mProgram); mProgram = 0; }
}

bool Renderer3D::init(int width, int height) {
    mWidth = width;
    mHeight = height;

    if (!createShaders()) return false;
    if (!createFBO()) return false;

    GL::glGenVertexArrays(1, &mVAO);
    GL::glGenBuffers(1, &mVBO);

    GL::glBindVertexArray(mVAO);
    GL::glBindBuffer(GL_ARRAY_BUFFER, mVBO);

    // Position attrib (location = 0)
    GL::glEnableVertexAttribArray(0);
    GL::glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, x));

    // Color attrib (location = 1)
    GL::glEnableVertexAttribArray(1);
    GL::glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex3D), (void*)offsetof(Vertex3D, r));

    GL::glBindVertexArray(0);
    return true;
}

void Renderer3D::resize(int width, int height) {
    if (width <= 0 || height <= 0) return;
    if (width == mWidth && height == mHeight) return;
    mWidth = width;
    mHeight = height;
    destroyFBO();
    createFBO();
}

bool Renderer3D::createFBO() {
    GL::glGenFramebuffers(1, &mFBO);
    GL::glBindFramebuffer(GL_FRAMEBUFFER, mFBO);

    // Color texture
    glGenTextures(1, &mColorTex);
    glBindTexture(GL_TEXTURE_2D, mColorTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, mWidth, mHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    GL::glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mColorTex, 0);

    // Depth renderbuffer
    GL::glGenRenderbuffers(1, &mDepthRBO);
    GL::glBindRenderbuffer(GL_RENDERBUFFER, mDepthRBO);
    GL::glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, mWidth, mHeight);
    GL::glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, mDepthRBO);

    bool status = (GL::glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
    GL::glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return status;
}

void Renderer3D::destroyFBO() {
    if (mFBO) { GL::glDeleteFramebuffers(1, &mFBO); mFBO = 0; }
    if (mColorTex) { glDeleteTextures(1, &mColorTex); mColorTex = 0; }
    if (mDepthRBO) { GL::glDeleteRenderbuffers(1, &mDepthRBO); mDepthRBO = 0; }
}

bool Renderer3D::createShaders() {
    const char* vShaderSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uMVP;
out vec4 vColor;
void main() {
    vColor = aColor;
    gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

    const char* fShaderSrc = R"(
#version 330 core
in vec4 vColor;
out vec4 FragColor;
void main() {
    FragColor = vColor;
}
)";

    GLuint vShader = GL::glCreateShader(GL_VERTEX_SHADER);
    GL::glShaderSource(vShader, 1, &vShaderSrc, nullptr);
    GL::glCompileShader(vShader);

    GLuint fShader = GL::glCreateShader(GL_FRAGMENT_SHADER);
    GL::glShaderSource(fShader, 1, &fShaderSrc, nullptr);
    GL::glCompileShader(fShader);

    mProgram = GL::glCreateProgram();
    GL::glAttachShader(mProgram, vShader);
    GL::glAttachShader(mProgram, fShader);
    GL::glLinkProgram(mProgram);

    GL::glDeleteShader(vShader);
    GL::glDeleteShader(fShader);

    mUniformMVP = GL::glGetUniformLocation(mProgram, "uMVP");
    return (mProgram != 0);
}

void Renderer3D::beginFrame() {
    mLineVertices.clear();

    GL::glBindFramebuffer(GL_FRAMEBUFFER, mFBO);
    glViewport(0, 0, mWidth, mHeight);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glClearColor(0.12f, 0.13f, 0.16f, 1.0f); // Dark elegant slate background
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer3D::endFrame() {
    if (!mLineVertices.empty()) {
        float mvp[16];
        mCamera.getMatrices(mWidth, mHeight, mvp);

        GL::glUseProgram(mProgram);
        GL::glUniformMatrix4fv(mUniformMVP, 1, GL_FALSE, mvp);

        GL::glBindVertexArray(mVAO);
        GL::glBindBuffer(GL_ARRAY_BUFFER, mVBO);
        GL::glBufferData(GL_ARRAY_BUFFER, mLineVertices.size() * sizeof(Vertex3D), mLineVertices.data(), GL_DYNAMIC_DRAW);

        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(mLineVertices.size()));

        GL::glBindVertexArray(0);
        GL::glUseProgram(0);
    }

    GL::glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Renderer3D::addLine(const Vector3& p0, const Vector3& p1, const Vector3& color, float alpha) {
    mLineVertices.push_back({ p0.x, p0.y, p0.z, color.x, color.y, color.z, alpha });
    mLineVertices.push_back({ p1.x, p1.y, p1.z, color.x, color.y, color.z, alpha });
}

void Renderer3D::drawGrid(float size, float step) {
    Vector3 gridColor(0.25f, 0.28f, 0.35f);
    Vector3 centerColor(0.4f, 0.45f, 0.55f);

    for (float x = -size; x <= size; x += step) {
        Vector3 c = (std::abs(x) < 0.001f) ? centerColor : gridColor;
        addLine(Vector3(x, 0.0f, -size), Vector3(x, 0.0f, size), c, 0.7f);
    }
    for (float z = -size; z <= size; z += step) {
        Vector3 c = (std::abs(z) < 0.001f) ? centerColor : gridColor;
        addLine(Vector3(-size, 0.0f, z), Vector3(size, 0.0f, z), c, 0.7f);
    }
}

void Renderer3D::drawAxes(float length) {
    addLine(Vector3(0, 0, 0), Vector3(length, 0, 0), Vector3(1.0f, 0.2f, 0.2f)); // X: Red
    addLine(Vector3(0, 0, 0), Vector3(0, length, 0), Vector3(0.2f, 1.0f, 0.2f)); // Y: Green
    addLine(Vector3(0, 0, 0), Vector3(0, 0, length), Vector3(0.2f, 0.4f, 1.0f)); // Z: Blue
}

void Renderer3D::drawWireBox(const Pose& pose, const Vector3& halfExtents, const Vector3& color) {
    Vector3 corners[8] = {
        Vector3(-halfExtents.x, -halfExtents.y, -halfExtents.z),
        Vector3( halfExtents.x, -halfExtents.y, -halfExtents.z),
        Vector3( halfExtents.x,  halfExtents.y, -halfExtents.z),
        Vector3(-halfExtents.x,  halfExtents.y, -halfExtents.z),
        Vector3(-halfExtents.x, -halfExtents.y,  halfExtents.z),
        Vector3( halfExtents.x, -halfExtents.y,  halfExtents.z),
        Vector3( halfExtents.x,  halfExtents.y,  halfExtents.z),
        Vector3(-halfExtents.x,  halfExtents.y,  halfExtents.z)
    };

    Vector3 worldCorners[8];
    for (int i = 0; i < 8; ++i) {
        worldCorners[i] = pose * corners[i];
    }

    // 12 edges
    const int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    for (int i = 0; i < 12; ++i) {
        addLine(worldCorners[edges[i][0]], worldCorners[edges[i][1]], color);
    }
}

void Renderer3D::drawWireSphere(const Pose& pose, float radius, const Vector3& color, int segments) {
    // XY circle
    for (int i = 0; i < segments; ++i) {
        float a0 = (i / static_cast<float>(segments)) * 6.2831853f;
        float a1 = ((i + 1) / static_cast<float>(segments)) * 6.2831853f;
        Vector3 p0 = pose * Vector3(std::cos(a0) * radius, std::sin(a0) * radius, 0.0f);
        Vector3 p1 = pose * Vector3(std::cos(a1) * radius, std::sin(a1) * radius, 0.0f);
        addLine(p0, p1, color);
    }
    // XZ circle
    for (int i = 0; i < segments; ++i) {
        float a0 = (i / static_cast<float>(segments)) * 6.2831853f;
        float a1 = ((i + 1) / static_cast<float>(segments)) * 6.2831853f;
        Vector3 p0 = pose * Vector3(std::cos(a0) * radius, 0.0f, std::sin(a0) * radius);
        Vector3 p1 = pose * Vector3(std::cos(a1) * radius, 0.0f, std::sin(a1) * radius);
        addLine(p0, p1, color);
    }
    // YZ circle
    for (int i = 0; i < segments; ++i) {
        float a0 = (i / static_cast<float>(segments)) * 6.2831853f;
        float a1 = ((i + 1) / static_cast<float>(segments)) * 6.2831853f;
        Vector3 p0 = pose * Vector3(0.0f, std::cos(a0) * radius, std::sin(a0) * radius);
        Vector3 p1 = pose * Vector3(0.0f, std::cos(a1) * radius, std::sin(a1) * radius);
        addLine(p0, p1, color);
    }
}

void Renderer3D::drawWireCapsule(const Pose& pose, float radius, float height, const Vector3& color, int segments) {
    float halfH = height * 0.5f;

    // Top sphere
    drawWireSphere(Pose(pose * Vector3(0, halfH, 0), pose.orientation), radius, color, segments);
    // Bottom sphere
    drawWireSphere(Pose(pose * Vector3(0, -halfH, 0), pose.orientation), radius, color, segments);

    // Connecting side lines
    addLine(pose * Vector3(-radius, -halfH, 0), pose * Vector3(-radius, halfH, 0), color);
    addLine(pose * Vector3( radius, -halfH, 0), pose * Vector3( radius, halfH, 0), color);
    addLine(pose * Vector3(0, -halfH, -radius), pose * Vector3(0, halfH, -radius), color);
    addLine(pose * Vector3(0, -halfH,  radius), pose * Vector3(0, halfH,  radius), color);
}

void Renderer3D::drawWireMesh(const Pose& pose, const Vector3& color) {
    // 3D Piramit Temsili Tel Kafes (Pyramid wireframe)
    Vector3 v0 = pose * Vector3(-1.0f, 0.0f, -1.0f);
    Vector3 v1 = pose * Vector3( 1.0f, 0.0f, -1.0f);
    Vector3 v2 = pose * Vector3( 0.0f, 0.0f,  1.0f);
    Vector3 apex = pose * Vector3(0.0f, 2.0f, 0.0f);
    addLine(v0, v1, color);
    addLine(v1, v2, color);
    addLine(v2, v0, color);
    addLine(v0, apex, color);
    addLine(v1, apex, color);
    addLine(v2, apex, color);
}

void Renderer3D::drawRaycast(const Vector3& origin, const Vector3& end, bool hasHit, const Vector3& hitPoint, const Vector3& hitNormal) {
    // Parlak lazer cizgisi (Cyan / Turkuaz)
    Vector3 rayColor = hasHit ? Vector3(0.0f, 1.0f, 0.9f) : Vector3(1.0f, 0.25f, 0.25f);
    addLine(origin, hasHit ? hitPoint : end, rayColor);

    if (hasHit) {
        // Vurus noktasi belirteci (Sari Arti Isareti)
        float s = 0.25f;
        Vector3 markerColor(1.0f, 0.9f, 0.1f);
        addLine(hitPoint + Vector3(-s, 0, 0), hitPoint + Vector3(s, 0, 0), markerColor);
        addLine(hitPoint + Vector3(0, -s, 0), hitPoint + Vector3(0, s, 0), markerColor);
        addLine(hitPoint + Vector3(0, 0, -s), hitPoint + Vector3(0, 0, s), markerColor);

        // Yuzey dikmesi (Normal vektoru: Parlak Yesil)
        addLine(hitPoint, hitPoint + hitNormal * 1.2f, Vector3(0.2f, 1.0f, 0.3f));
    }
}

void Renderer3D::drawContactPoint(const Vector3& pt, const Vector3& normal, float penetration, float normalImpulse) {
    // Small diamond marker
    float s = 0.08f;
    Vector3 markerColor(1.0f, 0.85f, 0.0f); // Bright yellow
    addLine(pt + Vector3(-s, 0, 0), pt + Vector3(s, 0, 0), markerColor);
    addLine(pt + Vector3(0, -s, 0), pt + Vector3(0, s, 0), markerColor);
    addLine(pt + Vector3(0, 0, -s), pt + Vector3(0, 0, s), markerColor);

    // Normal arrow
    float arrowLen = std::max(0.3f, std::min(1.5f, 0.3f + normalImpulse * 0.05f));
    Vector3 normEnd = pt + normal * arrowLen;
    addLine(pt, normEnd, Vector3(1.0f, 0.2f, 0.2f)); // Red normal arrow
}

void Renderer3D::drawVelocityVector(const Vector3& origin, const Vector3& velocity, const Vector3& color) {
    float speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
    if (speed < 0.01f) return;

    Vector3 velEnd = origin + velocity * 0.1f; // Scaled for visibility
    addLine(origin, velEnd, color);
}

} // namespace Baryon::Debugger
