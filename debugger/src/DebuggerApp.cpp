/*
 * Baryon Visual Debugger & Diagnostic Tool
 * Copyright (C) 2026 Kerim Aslan
 */

#include "bdebugger/DebuggerApp.hpp"
#include "bdebugger/GLFunctions.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include <iostream>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace Baryon::Debugger {

static void glfwErrorCallback(int error, const char* description) {
    std::cerr << "[GLFW Error " << error << "]: " << description << std::endl;
}

DebuggerApp::DebuggerApp() {
    mSimulator = std::make_unique<Simulator>();
    mSimulator->setCollisionListener(&mEventLogger);
}

DebuggerApp::~DebuggerApp() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (mWindow) {
        glfwDestroyWindow(mWindow);
        mWindow = nullptr;
    }
    glfwTerminate();
}

bool DebuggerApp::init() {
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // OpenGL 3.3 Core profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    mWindow = glfwCreateWindow(mWindowWidth, mWindowHeight, "Baryon Fizik Teshis Araci & Gorsel Hata Ayiklayici", nullptr, nullptr);
    if (!mWindow) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(mWindow);
    glfwSwapInterval(1); // Enable VSync

    // Initialize custom zero-dependency OpenGL function pointers
    if (!GL::initFunctions((void*(*)(const char*))glfwGetProcAddress)) {
        std::cerr << "Failed to load OpenGL 3.3 core functions via GLFW proc address!" << std::endl;
        return false;
    }

    // Initialize 3D Renderer with FBO
    if (!mRenderer.init(mViewportWidth, mViewportHeight)) {
        std::cerr << "Failed to initialize 3D Renderer!" << std::endl;
        return false;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Premium Dark Theme styling
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 6.0f;
    style.ChildRounding = 4.0f;
    style.FrameRounding = 4.0f;
    style.PopupRounding = 4.0f;
    style.ScrollbarRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.TabRounding = 4.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.11f, 0.14f, 1.0f);
    colors[ImGuiCol_Header] = ImVec4(0.20f, 0.25f, 0.35f, 1.0f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.28f, 0.35f, 0.48f, 1.0f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.24f, 0.30f, 0.42f, 1.0f);
    colors[ImGuiCol_Button] = ImVec4(0.22f, 0.26f, 0.36f, 1.0f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.30f, 0.38f, 0.52f, 1.0f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.18f, 0.22f, 0.30f, 1.0f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.15f, 0.17f, 0.22f, 1.0f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.22f, 0.25f, 0.32f, 1.0f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.25f, 0.30f, 0.40f, 1.0f);
    colors[ImGuiCol_Tab] = ImVec4(0.15f, 0.17f, 0.22f, 1.0f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.28f, 0.35f, 0.48f, 1.0f);
    colors[ImGuiCol_TabActive] = ImVec4(0.24f, 0.30f, 0.42f, 1.0f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.12f, 0.13f, 0.17f, 1.0f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.16f, 0.18f, 0.24f, 1.0f);

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(mWindow, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Load initial scene
    loadScene(mCurrentScene);
    setStatus("Initialized Baryon Visual Debugger & Diagnostic Engine.");
    return true;
}

void DebuggerApp::loadScene(SceneType scene) {
    mCurrentScene = scene;
    mRecorder.reset();
    mEventLogger.logItems.clear();
    mSelectedEntityId = 0xFFFFFFFF;
    TestScenes::loadScene(*mSimulator, scene);
    mSimulator->setCollisionListener(&mEventLogger);

    // Capture initial frame 0
    mRecorder.captureStep(*mSimulator, mSimulator->getFixedTimeStep());

    // Otomatik olarak ilk dinamik nesneyi sec (Kontroller hemen gorunur olsun!)
    const auto* f = mRecorder.getLatestFrame();
    if (f) {
        for (const auto& e : f->entities) {
            if (e.bodyType == 2) { // Dinamik
                mSelectedEntityId = e.id;
                break;
            }
        }
    }

    setStatus(std::string("Yuklendi: ") + TestScenes::getSceneName(scene));
}

void DebuggerApp::setStatus(const std::string& msg) {
    mStatusMessage = msg;
    mStatusTimer = 5.0f; // Show for 5 seconds
}

void DebuggerApp::run() {
    double lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(mWindow)) {
        glfwPollEvents();

        double currentTime = glfwGetTime();
        float dt = static_cast<float>(currentTime - lastTime);
        lastTime = currentTime;

        if (mStatusTimer > 0.0f) {
            mStatusTimer -= dt;
        }

        processInput(dt);
        updatePhysics(dt);
        render3DView();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        renderUI();

        // Rendering
        ImGui::Render();
        int displayW, displayH;
        glfwGetFramebufferSize(mWindow, &displayW, &displayH);
        glViewport(0, 0, displayW, displayH);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(mWindow);
    }
}

void DebuggerApp::processInput(float dt) {
    if (!mIsViewportHovered) return;

    ImGuiIO& io = ImGui::GetIO();

    // Zoom
    if (io.MouseWheel != 0.0f) {
        mRenderer.getCamera().zoom(io.MouseWheel * 1.5f);
    }

    // Orbit (Left click drag)
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        mRenderer.getCamera().orbit(io.MouseDelta.x * 0.4f, -io.MouseDelta.y * 0.4f);
    }

    // Pan (Right click or Middle click drag)
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right) || ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
        float factor = mRenderer.getCamera().distance * 0.002f;
        mRenderer.getCamera().pan(io.MouseDelta.x * factor, io.MouseDelta.y * factor);
    }
}

void DebuggerApp::updatePhysics(float dt) {
    if (mCurrentScene == SceneType::RaycastShowcase && mRaycastAutoSweep) {
        mRaycastAngle += 25.0f * dt;
        if (mRaycastAngle > 35.0f) mRaycastAngle = -35.0f;
    }

    if (mMode == AppMode::LiveSimulation) {
        if (!mIsPaused || mStepOnce) {
            float fixedDt = mSimulator->getFixedTimeStep();
            mSimulator->step(fixedDt * mSimSpeed);
            mRecorder.captureStep(*mSimulator, fixedDt);
            mStepOnce = false;
        }
    }
}

void DebuggerApp::render3DView() {
    mRenderer.beginFrame();

    if (mShowGrid) mRenderer.drawGrid(30.0f, 1.0f);
    if (mShowAxes) mRenderer.drawAxes(2.0f);

    // Fetch active frame depending on mode
    const DumpFrame* activeFrame = nullptr;
    if (mMode == AppMode::LiveSimulation) {
        activeFrame = mRecorder.getLatestFrame();
    } else {
        activeFrame = mReader.getCurrentFrame();
    }

    if (activeFrame) {
        // Draw Bodies
        if (mShowShapes) {
            for (const auto& e : activeFrame->entities) {
                Pose pose(Vector3(e.posX, e.posY, e.posZ), Quaternion(e.rotX, e.rotY, e.rotZ, e.rotW));
                Vector3 color;

                if (e.id == mSelectedEntityId) {
                    color = Vector3(1.0f, 0.9f, 0.1f); // Selected: Vibrant Yellow
                } else if (e.isTrigger) {
                    color = Vector3(0.95f, 0.35f, 0.85f); // Trigger Zone: Vibrant Magenta
                } else if (e.layer == 0x0002) {
                    color = Vector3(0.95f, 0.25f, 0.25f); // Kırmızı Takım (Red Team)
                } else if (e.layer == 0x0004) {
                    color = Vector3(0.25f, 0.55f, 0.95f); // Mavi Takım (Blue Team)
                } else if (e.bodyType == 0) {
                    color = Vector3(0.45f, 0.48f, 0.55f); // Static: Slate Grey
                } else if (e.isSleeping) {
                    color = Vector3(0.25f, 0.55f, 0.75f); // Sleeping: Calm Blue
                } else {
                    color = Vector3(0.2f, 0.85f, 0.45f); // Dynamic Active: Fresh Emerald Green
                }

                if (e.shapeKind == ShapeKind::Box) {
                    mRenderer.drawWireBox(pose, Vector3(e.shapeDimX, e.shapeDimY, e.shapeDimZ), color);
                } else if (e.shapeKind == ShapeKind::Sphere) {
                    mRenderer.drawWireSphere(pose, e.shapeDimX, color);
                } else if (e.shapeKind == ShapeKind::Capsule) {
                    mRenderer.drawWireCapsule(pose, e.shapeDimX, e.shapeDimY, color);
                } else if (e.shapeKind == ShapeKind::StaticMesh) {
                    mRenderer.drawWireMesh(pose, color);
                }

                if (mShowVelocities && e.bodyType == 2 && !e.isSleeping) {
                    mRenderer.drawVelocityVector(Vector3(e.posX, e.posY, e.posZ), 
                                                Vector3(e.linVelX, e.linVelY, e.linVelZ), 
                                                Vector3(0.2f, 0.8f, 1.0f));
                }
            }
        }

        // Draw Contacts
        if (mShowContacts) {
            for (const auto& c : activeFrame->contacts) {
                mRenderer.drawContactPoint(Vector3(c.pointX, c.pointY, c.pointZ),
                                           Vector3(c.normalX, c.normalY, c.normalZ),
                                           c.penetration, c.normalImpulse);
            }
        }
    }

    // Canli 3D Lazer Isini (Raycast Showcase)
    if (mCurrentScene == SceneType::RaycastShowcase && mMode == AppMode::LiveSimulation) {
        float rad = mRaycastAngle * 3.14159265f / 180.0f;
        Vector3 rayOrigin(0.0f, 2.0f, -12.0f);
        Vector3 rayDir(std::sin(rad), 0.0f, std::cos(rad));
        float maxDist = 25.0f;

        auto hit = mSimulator->raycast(rayOrigin, rayDir, maxDist);
        Vector3 endPoint = rayOrigin + rayDir * maxDist;
        Vector3 hitPt = rayOrigin + rayDir * hit.distance;
        mRenderer.drawRaycast(rayOrigin, endPoint, hit.hasHit, hitPt, hit.normal);
    }

    mRenderer.endFrame();
}

void DebuggerApp::renderUI() {
    renderMenuBar();

    ImGuiIO& io = ImGui::GetIO();
    float menuHeight = 24.0f;
    float leftWidth = 320.0f;
    float rightWidth = 380.0f;
    float bottomHeight = 240.0f;

    float availableW = io.DisplaySize.x;
    float availableH = std::max(200.0f, io.DisplaySize.y - menuHeight);

    float centerWidth = std::max(100.0f, availableW - leftWidth - rightWidth);
    float topHeight = std::max(100.0f, availableH - bottomHeight);

    // 1. Control Panel (Left column full height)
    ImGui::SetNextWindowPos(ImVec2(0.0f, menuHeight), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(leftWidth, availableH), ImGuiCond_Always);
    renderControlPanel();

    // 2. 3D Viewport (Center top)
    ImGui::SetNextWindowPos(ImVec2(leftWidth, menuHeight), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(centerWidth, topHeight), ImGuiCond_Always);
    renderViewportPanel();

    // 3. Diagnostics Panel (Right top)
    ImGui::SetNextWindowPos(ImVec2(leftWidth + centerWidth, menuHeight), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rightWidth, topHeight), ImGuiCond_Always);
    renderDiagnosticPanel();

    // 4. Entity Inspector (Center bottom)
    ImGui::SetNextWindowPos(ImVec2(leftWidth, menuHeight + topHeight), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(centerWidth, bottomHeight), ImGuiCond_Always);
    renderInspectorPanel();

    // 5. Timeline Scrubber (Right bottom)
    ImGui::SetNextWindowPos(ImVec2(leftWidth + centerWidth, menuHeight + topHeight), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(rightWidth, bottomHeight), ImGuiCond_Always);
    renderTimelinePanel();
}

void DebuggerApp::renderMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Dosya")) {
            if (ImGui::MenuItem("Kara Kutu Dokumu Kaydet (.baryon_dump)")) {
                if (mRecorder.saveBufferToFile("blackbox_dump.baryon_dump", mSimulator->getFixedTimeStep())) {
                    setStatus("Halka arabellek 'blackbox_dump.baryon_dump' dosyasina kaydedildi!");
                } else {
                    setStatus("Hata: Dokum dosyasi kaydedilemedi.");
                }
            }
            if (ImGui::MenuItem("Tamponu JSON Olarak Disa Aktar (.json)")) {
                if (mRecorder.exportBufferToJson("baryon_telemetry.json")) {
                    setStatus("Halka arabellek 'baryon_telemetry.json' dosyasina aktarildi!");
                } else {
                    setStatus("Hata: JSON dosyasina aktarilamadi.");
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Kayit Oynat / Yukle (.baryon_dump)")) {
                if (mReader.loadFromFile("blackbox_dump.baryon_dump")) {
                    mMode = AppMode::ReplayPlayback;
                    setStatus("Kayit dosyasi yuklendi: blackbox_dump.baryon_dump");
                } else {
                    setStatus("Hata: 'blackbox_dump.baryon_dump' dosyasi acilamadi.");
                }
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Cikis")) {
                glfwSetWindowShouldClose(mWindow, GLFW_TRUE);
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Gorunum")) {
            ImGui::MenuItem("Zemin Izgarasi", nullptr, &mShowGrid);
            ImGui::MenuItem("Koordinat Eksenleri", nullptr, &mShowAxes);
            ImGui::MenuItem("Tel Kafes Sekilleri", nullptr, &mShowShapes);
            ImGui::MenuItem("Temas Noktalari & Normaller", nullptr, &mShowContacts);
            ImGui::MenuItem("Hiz Vektorleri", nullptr, &mShowVelocities);
            ImGui::EndMenu();
        }

        // Status text right-aligned in menu bar
        ImGui::SameLine(ImGui::GetWindowWidth() - 400);
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "%s", mStatusMessage.c_str());

        ImGui::EndMainMenuBar();
    }
}

void DebuggerApp::renderControlPanel() {
    ImGui::Begin("Simulasyon & Sahneler", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    // Mode Toggle
    ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.9f, 1.0f), "CALISMA MODU:");
    if (ImGui::RadioButton("Canli Simulasyon", mMode == AppMode::LiveSimulation)) {
        mMode = AppMode::LiveSimulation;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Kayit Oynatici (Replay)", mMode == AppMode::ReplayPlayback)) {
        mMode = AppMode::ReplayPlayback;
    }

    ImGui::Separator();

    if (mMode == AppMode::LiveSimulation) {
        ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.5f, 1.0f), "HAZIR TEST SAHNELERI:");
        for (int i = 0; i < static_cast<int>(SceneType::Count); ++i) {
            SceneType st = static_cast<SceneType>(i);
            bool isCurrent = (mCurrentScene == st);
            if (ImGui::Selectable(TestScenes::getSceneName(st), isCurrent)) {
                loadScene(st);
            }
        }

        if (mCurrentScene == SceneType::RaycastShowcase) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.9f, 1.0f), "3D CANLI LAZER ISIN KONTROLLERI:");
            ImGui::Checkbox("Otomatik Taramayi Baslat/Durdur", &mRaycastAutoSweep);
            ImGui::SliderFloat("Lazer Acisi", &mRaycastAngle, -35.0f, 35.0f, "%.1f deg");
        } else if (mCurrentScene == SceneType::RevoluteHinge) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.2f, 1.0f), "MENTESE KAPI KONTROLLERI:");
            if (ImGui::Button("Kapiya Donus Torku Ver (+25 Nm)")) {
                auto entities = mSimulator->getRegistry().getComponentPool<Core::BodyState>().getAllEntities();
                for (auto e : entities) {
                    if (mSimulator->getRegistry().getComponent<Core::BodyState>(e).type == Core::BodyType::Dynamic) {
                        Body b(e, mSimulator->getRegistry());
                        b.applyTorque(Vector3(0.0f, 25.0f, 0.0f));
                    }
                }
                setStatus("Menteseli kapiya 25 Nm dikey tork uygulandi.");
            }
            ImGui::SameLine();
            if (ImGui::Button("Acisal Itme Ver")) {
                auto entities = mSimulator->getRegistry().getComponentPool<Core::BodyState>().getAllEntities();
                for (auto e : entities) {
                    if (mSimulator->getRegistry().getComponent<Core::BodyState>(e).type == Core::BodyType::Dynamic) {
                        Body b(e, mSimulator->getRegistry());
                        b.applyAngularImpulse(Vector3(0.0f, 15.0f, 0.0f));
                    }
                }
                setStatus("Menteseli kapiya 15 Nm*s acisal itme uygulandi.");
            }
        }

        ImGui::Separator();
        ImGui::TextWrapped("%s", TestScenes::getSceneDescription(mCurrentScene));
        ImGui::Separator();

        // Play / Pause / Step Controls
        if (mIsPaused) {
            if (ImGui::Button("Simulasyonu Baslat", ImVec2(140, 28))) mIsPaused = false;
        } else {
            if (ImGui::Button("Durdur", ImVec2(140, 28))) mIsPaused = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("1 Kare Ilerlet", ImVec2(100, 28))) {
            mIsPaused = true;
            mStepOnce = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Sahneyi Sifirla", ImVec2(100, 28))) {
            loadScene(mCurrentScene);
        }

        ImGui::SliderFloat("Hiz Carpani", &mSimSpeed, 0.1f, 3.0f, "%.1fx");

        // HER ZAMAN GORUNUR KUVVET, TORK VE BODY KONTROLLERI
        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.2f, 0.95f, 0.85f, 1.0f), "HIZLI KUVVET & TORK KONTROLLERI:");
        if (mSelectedEntityId != 0xFFFFFFFF && mSimulator->getRegistry().isAlive(ecs::Entity{mSelectedEntityId})) {
            Body b(ecs::Entity{mSelectedEntityId}, mSimulator->getRegistry());
            ImGui::Text("Secili Nesne: Varlik #%u (Sariyla Isaretli)", mSelectedEntityId);

            if (ImGui::Button("Yukari Kaldir (+35N Y)")) {
                b.applyForce(Vector3(0.0f, 35.0f, 0.0f));
                setStatus("Varlik #" + std::to_string(mSelectedEntityId) + " nesnesine +35N yukari kuvvet uygulandi.");
            }
            ImGui::SameLine();
            if (ImGui::Button("Ileri It (+30N X)")) {
                b.applyForce(Vector3(30.0f, 0.0f, 0.0f));
                setStatus("Varlik #" + std::to_string(mSelectedEntityId) + " nesnesine +30N ileri kuvvet uygulandi.");
            }

            if (ImGui::Button("Tork Uygula (+25 Nm Y)")) {
                b.applyTorque(Vector3(0.0f, 25.0f, 0.0f));
                setStatus("Varlik #" + std::to_string(mSelectedEntityId) + " nesnesine 25 Nm tork uygulandi (Donuyor!).");
            }
            ImGui::SameLine();
            if (ImGui::Button("Acisal Itme (+15 Nm*s)")) {
                b.applyAngularImpulse(Vector3(0.0f, 15.0f, 0.0f));
                setStatus("Varlik #" + std::to_string(mSelectedEntityId) + " nesnesine acisal itme uygulandi.");
            }

            if (ImGui::Button("Donusu ve Hizi Sifirla")) {
                b.setLinearVelocity(Vector3(0.0f, 0.0f, 0.0f));
                b.setAngularVelocity(Vector3(0.0f, 0.0f, 0.0f));
                setStatus("Varlik #" + std::to_string(mSelectedEntityId) + " durduruldu.");
            }
            ImGui::SameLine();
            bool isTrig = b.isTrigger().value_or(false);
            if (ImGui::Button(isTrig ? "Kati Yap" : "Tetikleyici (Trigger) Yap")) {
                b.setTrigger(!isTrig);
                setStatus("Varlik #" + std::to_string(mSelectedEntityId) + " tetikleyici modu degistirildi.");
            }
        } else {
            ImGui::TextDisabled("Etkilesim icin alttaki tablodan bir cisim secin.");
        }

        ImGui::Separator();
        ImGui::TextColored(ImVec4(0.9f, 0.7f, 0.2f, 1.0f), "TELEMETRI KAYDI & DOKUM:");
        if (!mRecorder.isLiveRecording()) {
            if (ImGui::Button("Canli Dosya Kaydini Baslat", ImVec2(-1, 26))) {
                mRecorder.startLiveRecording("live_simulation.baryon_dump", mSimulator->getFixedTimeStep());
                setStatus("Canli dosya kaydi baslatildi: live_simulation.baryon_dump");
            }
        } else {
            if (ImGui::Button("Canli Dosya Kaydini Durdur", ImVec2(-1, 26))) {
                mRecorder.stopLiveRecording();
                setStatus("Canli dosya kaydi tamamlandi ve kapatildi.");
            }
        }

        if (ImGui::Button("Kara Kutu Dokumu Kaydet (.baryon_dump)", ImVec2(-1, 26))) {
            if (mRecorder.saveBufferToFile("blackbox_dump.baryon_dump", mSimulator->getFixedTimeStep())) {
                setStatus("Halka arabellek 'blackbox_dump.baryon_dump' dosyasina kaydedildi!");
            }
        }

        if (ImGui::Button("Tamponu JSON Olarak Disa Aktar (.json)", ImVec2(-1, 26))) {
            if (mRecorder.exportBufferToJson("baryon_telemetry.json")) {
                setStatus("Halka arabellek 'baryon_telemetry.json' dosyasina aktarildi!");
            }
        }
    } else {
        // Replay Mode Controls
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "KAYIT OYNATICI KONTROLLERI:");
        if (mReader.isLoaded()) {
            ImGui::Text("Yuklu Dosya: %s", mReader.getFilePath().c_str());
            ImGui::Text("Toplam Kare: %zu", mReader.getFrameCount());

            if (ImGui::Button("Onceki Kare (<)", ImVec2(120, 26))) mReader.stepBackward();
            ImGui::SameLine();
            if (ImGui::Button("Sonraki Kare (>)", ImVec2(120, 26))) mReader.stepForward();

            auto anomalyFrames = mReader.getAnomalyFrameIndices();
            if (!anomalyFrames.empty()) {
                if (ImGui::Button("Sonraki Anomaliye Atla", ImVec2(-1, 26))) {
                    for (size_t fIdx : anomalyFrames) {
                        if (fIdx > mReader.getCurrentIndex()) {
                            mReader.setCurrentIndex(fIdx);
                            break;
                        }
                    }
                }
            }
        } else {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Yuklu kayit dosyasi bulunamadi.");
            if (ImGui::Button("Load 'blackbox_dump.baryon_dump'", ImVec2(-1, 28))) {
                if (mReader.loadFromFile("blackbox_dump.baryon_dump")) {
                    setStatus("Yuklendi: blackbox_dump.baryon_dump");
                }
            }
        }
    }

    ImGui::End();
}

void DebuggerApp::renderViewportPanel() {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("3D Gorunum", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    ImVec2 viewSize = ImGui::GetContentRegionAvail();
    if (viewSize.x > 50 && viewSize.y > 50) {
        if (static_cast<int>(viewSize.x) != mRenderer.getWidth() ||
            static_cast<int>(viewSize.y) != mRenderer.getHeight()) {
            mRenderer.resize(static_cast<int>(viewSize.x), static_cast<int>(viewSize.y));
        }
    }

    mIsViewportHovered = ImGui::IsWindowHovered();

    // Render FBO texture into ImGui window
    ImTextureID texID = (ImTextureID)(uintptr_t)mRenderer.getTextureId();
    ImGui::Image(texID, viewSize, ImVec2(0, 1), ImVec2(1, 0));

    // Overlay HUD
    ImGui::SetCursorPos(ImVec2(12, 28));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0, 0, 0, 0.5f));
    ImGui::BeginChild("HUD", ImVec2(240, 105), true);
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

    const DumpFrame* f = (mMode == AppMode::LiveSimulation) ? mRecorder.getLatestFrame() : mReader.getCurrentFrame();
    if (f) {
        ImGui::Text("Kare: %llu | Zaman: %.2fs", f->frameIndex, f->simulationTime);
        ImGui::Text("Aktif: %u | Uykuda: %u", f->activeBodyCount, f->sleepingBodyCount);
        ImGui::Text("Kinetik Enerji: %.2f J", f->totalKineticEnergy);
        ImGui::Text("Temas Noktasi: %zu", f->contacts.size());
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();

    ImGui::End();
    ImGui::PopStyleVar();
}

void DebuggerApp::renderDiagnosticPanel() {
    ImGui::Begin("Anomali & Teshis Paneli", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    const DumpFrame* f = (mMode == AppMode::LiveSimulation) ? mRecorder.getLatestFrame() : mReader.getCurrentFrame();

    size_t totalAnomalies = f ? f->anomalies.size() : 0;
    if (totalAnomalies == 0) {
        ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.4f, 1.0f), "DURUM: OPTIMAL (Anomali Tespit Edilmedi)");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "DURUM: MEVCUT KAREDE %zu ANOMALI UYARISI!", totalAnomalies);
    }

    ImGui::Separator();

    if (f && !f->anomalies.empty()) {
        if (ImGui::BeginTable("AnomalyTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY)) {
            ImGui::TableSetupColumn("Onem", ImGuiTableColumnFlags_WidthFixed, 70.0f);
            ImGui::TableSetupColumn("Anomali Turu", ImGuiTableColumnFlags_WidthFixed, 140.0f);
            ImGui::TableSetupColumn("Varlik", ImGuiTableColumnFlags_WidthFixed, 50.0f);
            ImGui::TableSetupColumn("Aciklama", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            for (const auto& a : f->anomalies) {
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                if (a.severity == AnomalySeverity::Critical) {
                    ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "KRITIK");
                } else if (a.severity == AnomalySeverity::Warning) {
                    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.1f, 1.0f), "UYARI");
                } else {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "BILGI");
                }

                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%s", AnomalyRecord::typeToString(a.type));

                ImGui::TableSetColumnIndex(2);
                if (a.entityId != 0xFFFFFFFF) {
                    if (ImGui::SmallButton(std::to_string(a.entityId).c_str())) {
                        mSelectedEntityId = a.entityId;
                    }
                } else {
                    ImGui::Text("-");
                }

                ImGui::TableSetColumnIndex(3);
                ImGui::TextWrapped("%s", a.description.c_str());
            }
            ImGui::EndTable();
        }
    } else {
        ImGui::TextDisabled("Bu adimda hicbir uyari veya hata kaydedilmedi.");
    }

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.9f, 0.6f, 0.2f, 1.0f), "CANLI CARPISMA & TETIKLEYICI OLAYLARI (CollisionListener):");
    if (mEventLogger.logItems.empty()) {
        ImGui::TextDisabled("Henuz hicbir carpisma veya tetikleyici olayi gerceklesmedi.");
    } else {
        if (ImGui::BeginTable("EventsTable", 1, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(0, 100))) {
            for (auto it = mEventLogger.logItems.rbegin(); it != mEventLogger.logItems.rend(); ++it) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                if (it->isTrigger) {
                    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.9f, 1.0f), "%s", it->message.c_str());
                } else {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "%s", it->message.c_str());
                }
            }
            ImGui::EndTable();
        }
    }

    ImGui::End();
}

void DebuggerApp::renderInspectorPanel() {
    ImGui::Begin("Varlik Inceleyici (Entity Inspector)", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    const DumpFrame* f = (mMode == AppMode::LiveSimulation) ? mRecorder.getLatestFrame() : mReader.getCurrentFrame();

    if (!f || f->entities.empty()) {
        ImGui::TextDisabled("Mevcut sahnede hicbir varlik bulunmuyor.");
        ImGui::End();
        return;
    }

    if (ImGui::BeginTable("EntitiesTable", 6, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(0, 180))) {
        ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 40.0f);
        ImGui::TableSetupColumn("Tur", ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn("Konum (X, Y, Z)", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Hiz", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableSetupColumn("Kutle", ImGuiTableColumnFlags_WidthFixed, 50.0f);
        ImGui::TableSetupColumn("CCD", ImGuiTableColumnFlags_WidthFixed, 40.0f);
        ImGui::TableHeadersRow();

        for (const auto& e : f->entities) {
            ImGui::TableNextRow();
            bool isSelected = (e.id == mSelectedEntityId);

            ImGui::TableSetColumnIndex(0);
            if (ImGui::Selectable(std::to_string(e.id).c_str(), isSelected, ImGuiSelectableFlags_SpanAllColumns)) {
                mSelectedEntityId = e.id;
            }

            ImGui::TableSetColumnIndex(1);
            if (e.bodyType == 0) ImGui::Text("Statik");
            else if (e.bodyType == 1) ImGui::Text("Kinematik");
            else ImGui::Text(e.isSleeping ? "Uykuda" : "Dinamik");

            ImGui::TableSetColumnIndex(2);
            ImGui::Text("%.2f, %.2f, %.2f", e.posX, e.posY, e.posZ);

            ImGui::TableSetColumnIndex(3);
            float speed = std::sqrt(e.linVelX * e.linVelX + e.linVelY * e.linVelY + e.linVelZ * e.linVelZ);
            ImGui::Text("%.2f", speed);

            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%.1f", e.mass);

            ImGui::TableSetColumnIndex(5);
            ImGui::Text(e.useCCD ? "EVET" : "HAYIR");
        }
        ImGui::EndTable();
    }

    ImGui::Separator();

    // Selected Entity details
    if (mSelectedEntityId != 0xFFFFFFFF) {
        const DumpEntityRecord* sel = nullptr;
        for (const auto& e : f->entities) {
            if (e.id == mSelectedEntityId) { sel = &e; break; }
        }

        if (sel) {
            ImGui::TextColored(ImVec4(1.0f, 0.9f, 0.2f, 1.0f), "SECILI VARLIK #%u DETAYLARI:", sel->id);
            ImGui::Text("Konum: (%.3f, %.3f, %.3f)", sel->posX, sel->posY, sel->posZ);
            ImGui::Text("Donus (Quat): (%.3f, %.3f, %.3f, %.3f)", sel->rotX, sel->rotY, sel->rotZ, sel->rotW);
            ImGui::Text("Cizgisel Hiz: (%.3f, %.3f, %.3f) m/s", sel->linVelX, sel->linVelY, sel->linVelZ);
            ImGui::Text("Acisal Hiz: (%.3f, %.3f, %.3f) rad/s", sel->angVelX, sel->angVelY, sel->angVelZ);
            ImGui::Text("Kutle: %.2f kg | CCD: %s", sel->mass, sel->useCCD ? "Aktif" : "Pasif");
            ImGui::Text("Tetikleyici: %s | Katman: 0x%04X | Maske: 0x%04X", sel->isTrigger ? "EVET" : "HAYIR", sel->layer, sel->mask);

            // Interactive Body API testing
            if (mMode == AppMode::LiveSimulation && mSimulator->getRegistry().isAlive(ecs::Entity{sel->id})) {
                Body b(ecs::Entity{sel->id}, mSimulator->getRegistry());

                ImGui::Separator();
                ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.8f, 1.0f), "CANLI BODY API KONTROLLERI:");

                if (ImGui::Button("Yukari Kuvvet (+30N Y)")) {
                    b.applyForce(Vector3(0.0f, 30.0f, 0.0f));
                    setStatus("Varlik #" + std::to_string(sel->id) + " nesnesine +30N yukari kuvvet uygulandi.");
                }
                ImGui::SameLine();
                if (ImGui::Button("Ileri Kuvvet (+30N X)")) {
                    b.applyForce(Vector3(30.0f, 0.0f, 0.0f));
                    setStatus("Varlik #" + std::to_string(sel->id) + " nesnesine +30N X kuvvet uygulandi.");
                }

                if (ImGui::Button("Donus Torku (20N*m Y)")) {
                    b.applyTorque(Vector3(0.0f, 20.0f, 0.0f));
                    setStatus("Varlik #" + std::to_string(sel->id) + " nesnesine 20 N*m tork uygulandi.");
                }
                ImGui::SameLine();
                if (ImGui::Button("Acisal Itme (10 N*m*s)")) {
                    b.applyAngularImpulse(Vector3(0.0f, 10.0f, 0.0f));
                    setStatus("Varlik #" + std::to_string(sel->id) + " nesnesine acisal itme uygulandi.");
                }

                bool isTrig = b.isTrigger().value_or(false);
                if (isTrig) {
                    if (ImGui::Button("Tetikleyiciyi Kapat (Katilastir)")) {
                        b.setTrigger(false);
                        setStatus("Varlik #" + std::to_string(sel->id) + " kati yapildi.");
                    }
                } else {
                    if (ImGui::Button("Tetikleyici Yap (Hayalet Alan)")) {
                        b.setTrigger(true);
                        setStatus("Varlik #" + std::to_string(sel->id) + " tetikleyici (trigger) yapildi.");
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("Acisal Hizi Sifirla")) {
                    b.setAngularVelocity(Vector3(0.0f, 0.0f, 0.0f));
                }

                if (ImGui::SmallButton("Katman 1 (0x0002)")) b.setCollisionLayer(0x0002);
                ImGui::SameLine();
                if (ImGui::SmallButton("Katman 2 (0x0004)")) b.setCollisionLayer(0x0004);
                ImGui::SameLine();
                if (ImGui::SmallButton("Genel (0x0001)")) b.setCollisionLayer(0x0001);
            }
        }
    }

    ImGui::End();
}

void DebuggerApp::renderTimelinePanel() {
    ImGui::Begin("Zaman Cizelgesi & Oynatici", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    if (mMode == AppMode::LiveSimulation) {
        size_t bufSize = mRecorder.getRecordedFrameCount();
        ImGui::Text("Halka Arabellek: %zu / %zu kare RAM'de tutuluyor", bufSize, mRecorder.getBufferSize());
        ImGui::ProgressBar((float)bufSize / (float)mRecorder.getBufferSize(), ImVec2(-1, 18));
    } else {
        if (mReader.isLoaded()) {
            int currentIdx = static_cast<int>(mReader.getCurrentIndex());
            int maxIdx = static_cast<int>(mReader.getFrameCount() - 1);

            ImGui::Text("Oynatma Karesi: %d / %d (Zaman: %.3fs)", currentIdx, maxIdx, 
                        mReader.getCurrentFrame() ? mReader.getCurrentFrame()->simulationTime : 0.0f);

            if (ImGui::SliderInt("Zaman Cizelgesi", &currentIdx, 0, maxIdx)) {
                mReader.setCurrentIndex(static_cast<size_t>(currentIdx));
            }
        } else {
            ImGui::TextDisabled("Geri sarma ozelligini kullanmak icin bir .baryon_dump dosyasi yukleyin.");
        }
    }

    ImGui::End();
}

} // namespace Baryon::Debugger
