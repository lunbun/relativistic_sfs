add_library(glad STATIC
        external/glad/src/gl.c
)

target_include_directories(glad
        PUBLIC external/glad/include
)

find_package(glfw3 REQUIRED)
add_library(imgui STATIC
        external/imgui/imgui.cpp
        external/imgui/imgui_demo.cpp
        external/imgui/imgui_draw.cpp
        external/imgui/imgui_tables.cpp
        external/imgui/imgui_widgets.cpp
        external/imgui/backends/imgui_impl_glfw.cpp
        external/imgui/backends/imgui_impl_opengl3.cpp
        external/imgui/misc/cpp/imgui_stdlib.cpp
)
target_link_libraries(imgui
        PUBLIC glfw
)
target_include_directories(imgui
        PUBLIC external/imgui
)
