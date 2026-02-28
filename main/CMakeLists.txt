file(GLOB_RECURSE PROJECT_SOURCES "src/*.c" "*.c")

idf_component_register(SRCS ${PROJECT_SOURCES}
        INCLUDE_DIRS "." "src" "include")

target_compile_options(${COMPONENT_LIB} PRIVATE -O3 -ffast-math -ffp-contract=fast)
