cmake_minimum_required(VERSION 3.24)
project(relativistic_sfs)

add_executable(relativistic_sfs)

set_property(TARGET relativistic_sfs PROPERTY CXX_STANDARD 17)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(relativistic_sfs PRIVATE -O0 -g -march=native -fno-rtti)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(relativistic_sfs PRIVATE -O3 -DNDEBUG -march=native -flto=auto -fno-rtti)
endif()

find_package(glfw3 3.4 REQUIRED)

target_include_directories(relativistic_sfs PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")

# Usage: embed_file(<type> <path>)
# type: "binary" or "text"
function(embed_file type path)
    # Create a custom target that will generate the object file
    set(full_path "${CMAKE_SOURCE_DIR}/${path}")
    string(MAKE_C_IDENTIFIER "${path}" file_identifier)
    string(TOUPPER "${file_identifier}" upper_identifier)
    set(output "${CMAKE_BINARY_DIR}/embed/${file_identifier}.o")
    set(target "embed_${file_identifier}")

    set(output_format elf64-x86-64)
    if(type STREQUAL "binary")
        set(embed_command ${CMAKE_OBJCOPY} -I binary -O ${output_format} -B i386 ${full_path} ${output})
    elseif(type STREQUAL "text")
        # Hack to add a null byte at the end of the binary since it is text
        set(embed_command python -c "\"import os, sys\; size = os.path.getsize(sys.argv[1])\; os.system(f'\\\"${CMAKE_OBJCOPY}\\\" -I binary -O ${output_format} -B i386 --pad-to={size+1} {sys.argv[1]} {sys.argv[2]}')\"" ${full_path} ${output})
    else()
        message(FATAL_ERROR "Unknown embed type: ${type}")
    endif()

    message(STATUS "Embedding ${type} file ${path}")

    target_link_libraries(relativistic_sfs PRIVATE ${output})
    add_custom_command(
            OUTPUT ${output}
            COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/embed"
            COMMAND ${embed_command}
            DEPENDS ${path}
    )

    add_custom_target(${target} DEPENDS ${output})
    add_dependencies(relativistic_sfs ${target})

    # Generate the symbols to link to the embedded file, and add them as compile definitions
    string(MAKE_C_IDENTIFIER "${full_path}" full_identifier)
    set(file_start "_binary_${full_identifier}_start")
    set(file_end "_binary_${full_identifier}_end")
    set(macro_start "EMBED_START_${upper_identifier}")
    set(macro_end "EMBED_END_${upper_identifier}")
    target_compile_definitions(relativistic_sfs PRIVATE
            ${macro_start}=${file_start}
            ${macro_end}=${file_end}
    )
endfunction()

embed_file(text assets/dot_frag.glsl)
embed_file(text assets/dot_vert.glsl)
embed_file(text assets/traj_frag.glsl)
embed_file(text assets/traj_vert.glsl)

add_subdirectory(lib)

target_link_libraries(relativistic_sfs PRIVATE Eigen3::Eigen EnTT::EnTT glfw glad imgui)

add_subdirectory(src)
