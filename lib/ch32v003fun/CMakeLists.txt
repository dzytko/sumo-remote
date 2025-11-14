project(ch32v003fun C)
add_library(ch32v003fun ch32v003fun.c)
target_include_directories(ch32v003fun PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_options(
        ${CMAKE_PROJECT_NAME}
        PRIVATE
        -T ${CMAKE_CURRENT_LIST_DIR}/ch32v003fun.ld
)
