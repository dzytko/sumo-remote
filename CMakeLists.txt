cmake_minimum_required(VERSION 3.28)
project(sumo_remote C)

set(CMAKE_C_STANDARD 99)

add_compile_options(-O0 -g -march=rv32ec -mabi=ilp32e)

add_executable(sumo_remote main.c)

add_subdirectory(lib/ch32v003fun)

target_link_libraries(
        ${CMAKE_PROJECT_NAME}
        ch32v003fun
        gcc
)
target_link_options(
        ${CMAKE_PROJECT_NAME}
        PRIVATE
        -specs=nosys.specs
        -nostartfiles
        -nostdlib
        -march=rv32ec -mabi=ilp32e
)


add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD
        COMMAND size -G ${PROJECT_NAME}${CMAKE_EXECUTABLE_SUFFIX}
)