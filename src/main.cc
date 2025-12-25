#include <iostream>
#include <chrono>
#include <random>

#include <Eigen/Dense>
#include <entt/entt.hpp>
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

struct PhysicsState {
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
};

struct ForceAccumulator {
    Eigen::Vector3d force;
};

struct Body {
    double mass;
};

struct NumIntegrState { PhysicsState st; };
struct BodyState { PhysicsState st; };
struct TempState { PhysicsState st; };

struct RenderDotLarge { };
struct RenderDotSmall { };
struct RenderTrajectory {};

std::string formatDuration(std::chrono::seconds duration)
{
    using namespace std::chrono;

    long long secs = duration.count();

    const int seconds_per_minute = 60;
    const int seconds_per_hour   = 3600;
    const int seconds_per_day    = 86400;
    const int seconds_per_month  = 2592000;  // ~30 days
    const int seconds_per_year   = 31536000; // ~365 days

    long long years   = secs / seconds_per_year; secs %= seconds_per_year;
    long long months  = secs / seconds_per_month; secs %= seconds_per_month;
    long long days    = secs / seconds_per_day; secs %= seconds_per_day;
    long long hours   = secs / seconds_per_hour; secs %= seconds_per_hour;
    long long minutes = secs / seconds_per_minute; secs %= seconds_per_minute;
    long long seconds = secs;

    std::stringstream ss;
    if(years)   ss << years   << " years ";
    if(months)  ss << months  << " months ";
    if(days)    ss << days    << " days ";
    // if(hours)   ss << hours   << " hours ";
    // if(minutes) ss << minutes << " minutes ";
    // if(seconds) ss << seconds << " seconds";

    std::string result = ss.str();
    if(result.empty()) result = "0 seconds";
    return result;
}

void gravitySystem(entt::registry &registry) {
    constexpr double G = 6.67430e-11;

    auto view1 = registry.view<NumIntegrState, Body>();
    auto view2 = registry.view<ForceAccumulator, NumIntegrState, Body>();

    for (auto a : view1) {
        auto &physA = view1.get<NumIntegrState>(a);
        auto &bodyA = view1.get<Body>(a);
        for (auto b : view2) {
            if (a == b) continue;

            auto &physB = view2.get<NumIntegrState>(b);
            auto &bodyB = view2.get<Body>(b);
            auto &forceAcc = view2.get<ForceAccumulator>(b);

            Eigen::Vector3d r = physB.st.pos - physA.st.pos;
            double norm = r.norm();
            if (norm < 1e6) continue; // Avoid singularity

            forceAcc.force -= G * bodyA.mass * bodyB.mass * r / (norm * norm * norm);
        }
    }
}

void physicsUpdate(entt::registry &registry, double dt) {
    auto forcesView = registry.view<ForceAccumulator, NumIntegrState, Body>();
    for (auto entity : forcesView) {
        auto &forceAcc = forcesView.get<ForceAccumulator>(entity);
        forceAcc.force.setZero();
    }

    gravitySystem(registry);

    for (auto entity : forcesView) {
        auto &forceAcc = forcesView.get<ForceAccumulator>(entity);
        auto &physics = forcesView.get<NumIntegrState>(entity);
        auto &body = forcesView.get<Body>(entity);
        physics.st.vel += dt * forceAcc.force / body.mass;
        physics.st.pos += dt * physics.st.vel;
    }
}

template<typename From, typename To>
void syncState(entt::registry &registry) {
    auto view = registry.view<From, To>();
    for (auto entity : view) {
        auto &from = view.template get<From>(entity);
        auto &to = view.template get<To>(entity);
        to.st = from.st;
    }
}

constexpr double kRenderScale = 1.0 / 1.0e12;
void render(entt::registry &registry) {
    auto largeView = registry.view<BodyState, RenderDotLarge>();
    auto smallView = registry.view<BodyState, RenderDotSmall>();
    glPointSize(15.0f);
    glBegin(GL_POINTS);
    for (auto entity : largeView) {
        auto &body = largeView.get<BodyState>(entity);
        Eigen::Vector3d screenPos = body.st.pos * kRenderScale;
        glVertex2d(screenPos.x(), screenPos.y());
    }
    glEnd();

    glPointSize(5.0f);
    glBegin(GL_POINTS);
    for (auto entity : smallView) {
        auto &body = smallView.get<BodyState>(entity);
        Eigen::Vector3d screenPos = body.st.pos * kRenderScale;
        glVertex2d(screenPos.x(), screenPos.y());
    }
    glEnd();
}

void renderTrajectories(entt::registry &registry) {
    constexpr double dt = 1.0e5;
    constexpr int steps = 15;

    auto view = registry.view<BodyState, NumIntegrState, TempState, RenderTrajectory>();
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    syncState<BodyState, NumIntegrState>(registry);
    for (int i = 0; i < steps; i++) {
        syncState<NumIntegrState, TempState>(registry);
        physicsUpdate(registry, dt);
        for (auto entity : view) {
            auto &temp = view.get<TempState>(entity);
            auto &numIntegr = view.get<NumIntegrState>(entity);
            Eigen::Vector3d p1 = temp.st.pos * kRenderScale;
            Eigen::Vector3d p2 = numIntegr.st.pos * kRenderScale;
            glVertex2d(p1.x(), p1.y());
            glVertex2d(p2.x(), p2.y());
        }
    }
    glEnd();
}

int main() {
    std::cout << "Hello, World!" << std::endl;

    constexpr int WIDTH = 1600;
    constexpr int HEIGHT = 1200;

    // Init GLFW
    glfwInit();
    // Set all the required options for GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    // Create a GLFWwindow object that we can use for GLFW's functions
    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "Spaceflight Simulator", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    if (!window)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Set the required callback functions
    // glfwSetKeyCallback(window, key_callback);

    // Load OpenGL functions, gladLoadGL returns the loaded version, 0 on error.
    int version = gladLoadGL(glfwGetProcAddress);
    if (!version)
    {
        std::cout << "Failed to initialize OpenGL context" << std::endl;
        return -1;
    }

    // Successfully loaded OpenGL
    std::cout << "Loaded OpenGL " << GLAD_VERSION_MAJOR(version) << "." << GLAD_VERSION_MINOR(version) << std::endl;

    // Define the viewport dimensions
    glViewport(0, 0, WIDTH, HEIGHT);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.FontGlobalScale = 2.0f;

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);          // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
    ImGui_ImplOpenGL3_Init();

    int fps = 0;
    int frameCount = 0;
    std::chrono::time_point<std::chrono::steady_clock> lastFpsReading = std::chrono::steady_clock::now();

    entt::registry registry;
    auto sun = registry.create();
    registry.emplace<BodyState>(sun, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d::Zero());
    registry.emplace<Body>(sun, 1.989e30);
    registry.emplace<RenderDotLarge>(sun);

    auto mercury = registry.create();
    registry.emplace<BodyState>(mercury, Eigen::Vector3d(57.9e9, 0, 0), Eigen::Vector3d(0, 47870, 0));
    registry.emplace<Body>(mercury, 3.3011e23);
    registry.emplace<RenderDotLarge>(mercury);
    registry.emplace<RenderTrajectory>(mercury);

    auto venus = registry.create();
    registry.emplace<BodyState>(venus, Eigen::Vector3d(108.2e9, 0, 0), Eigen::Vector3d(0, 35020, 0));
    registry.emplace<Body>(venus, 4.8675e24);
    registry.emplace<RenderDotLarge>(venus);
    registry.emplace<RenderTrajectory>(venus);

    auto earth = registry.create();
    registry.emplace<BodyState>(earth, Eigen::Vector3d(149.6e9, 0, 0), Eigen::Vector3d(0, 29784.8, 0));
    registry.emplace<Body>(earth, 5.972e24);
    registry.emplace<RenderDotLarge>(earth);
    registry.emplace<RenderTrajectory>(earth);

    auto mars = registry.create();
    registry.emplace<BodyState>(mars, Eigen::Vector3d(227.9e9, 0, 0), Eigen::Vector3d(0, 24077, 0));
    registry.emplace<Body>(mars, 6.4171e23);
    registry.emplace<RenderDotLarge>(mars);
    registry.emplace<RenderTrajectory>(mars);

    auto jupiter = registry.create();
    registry.emplace<BodyState>(jupiter, Eigen::Vector3d(778.5e9, 0, 0), Eigen::Vector3d(0, 13070, 0));
    registry.emplace<Body>(jupiter, 1.8982e27);
    registry.emplace<RenderDotLarge>(jupiter);
    registry.emplace<RenderTrajectory>(jupiter);

    // Create asteroids
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distanceDistribution(300.0e9, 500.0e9);
    std::uniform_real_distribution<double> speedDistribution(15000.0, 25000.0);
    std::uniform_real_distribution<double> angleDistribution(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> massDistribution(1.0e15, 1.0e20);
    for (int i = 0; i < 100; i++) {
        double distance = distanceDistribution(generator);
        double speed = speedDistribution(generator);
        double angle = angleDistribution(generator);
        double mass = massDistribution(generator);

        Eigen::Vector3d position(distance * cos(angle), distance * sin(angle), 0);
        Eigen::Vector3d velocity(-speed * sin(angle), speed * cos(angle), 0);

        auto asteroid = registry.create();
        registry.emplace<BodyState>(asteroid, position, velocity);
        registry.emplace<Body>(asteroid, mass);
        registry.emplace<RenderDotSmall>(asteroid);
    }

    for (auto entity : registry.view<RenderTrajectory>()) {
        registry.emplace<TempState>(entity);
    }

    for (auto entity : registry.view<BodyState>()) {
        registry.emplace<NumIntegrState>(entity);
        if (entity != sun) registry.emplace<ForceAccumulator>(entity);
    }

    double time = 0.0;

    // Game loop
    while (!glfwWindowShouldClose(window))
    {
        // Check if any events have been activated (key pressed, mouse moved etc.) and call corresponding response functions
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Text("%d fps", fps);

        // Render
        // Clear the colorbuffer
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        constexpr double dt = 36000.0;
        syncState<BodyState, NumIntegrState>(registry);
        physicsUpdate(registry, dt);
        syncState<NumIntegrState, BodyState>(registry);
        render(registry);
        renderTrajectories(registry);

        time += dt;
        std::string formattedTime = formatDuration(std::chrono::seconds(static_cast<long long>(time)));
        ImGui::Text("Simulated time: %s", formattedTime.c_str());

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Swap the screen buffers
        glfwSwapBuffers(window);

        frameCount++;
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - lastFpsReading).count();
        if (duration >= 1) {
            fps = frameCount;
            frameCount = 0;
            lastFpsReading = now;
        }
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    // Terminates GLFW, clearing any resources allocated by GLFW.
    glfwTerminate();
    return 0;
}
