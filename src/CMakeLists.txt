add_subdirectory(model)
add_subdirectory(physics)
add_subdirectory(render)

target_sources(relativistic_sfs PRIVATE
        main.cc)
