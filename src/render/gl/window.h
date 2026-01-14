#pragma once

#include <chrono>
#include <memory>

// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
// clang-format on

class MainWindow {
public:
    // Creates window and initializes OpenGL and ImGui.
    //
    // Returns nullptr if window creation fails.
    static std::unique_ptr<MainWindow> create();

    MainWindow(GLFWwindow* window);
    ~MainWindow();

    void initImGui();

    GLFWwindow *handle() const { return window_; }
    int width() const;
    int height() const;
    bool shouldClose() const;

    void startFrame();
    void endFrame();

private:
    GLFWwindow* window_;
    int fps_;
    int frameCount_;
    std::chrono::time_point<std::chrono::steady_clock> lastFpsReading_;
};
