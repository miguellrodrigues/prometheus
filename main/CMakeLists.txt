file(GLOB_RECURSE PROJECT_SOURCES "src/*.c" "*.c")

idf_component_register(SRCS ${PROJECT_SOURCES}
        INCLUDE_DIRS "." "src" "include")