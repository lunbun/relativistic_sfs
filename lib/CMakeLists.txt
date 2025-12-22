add_library(glad STATIC
        external/glad/src/gl.c
)

target_include_directories(glad
        PUBLIC external/glad/include
)
