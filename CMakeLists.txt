cmake_minimum_required(VERSION 3.24)
project(relativistic_sfs)

add_executable(relativistic_sfs src/main.cc)

set_property(TARGET relativistic_sfs PROPERTY CXX_STANDARD 17)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(relativistic_sfs PRIVATE -O0 -g -march=native -fno-rtti)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(relativistic_sfs PRIVATE -Ofast -DNDEBUG -march=native -flto=auto -fno-rtti)
endif()

find_package(glfw3 3.4 REQUIRED)

target_include_directories(relativistic_sfs PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")

add_subdirectory(lib)

target_link_libraries(relativistic_sfs glfw glad)

add_subdirectory(src)
