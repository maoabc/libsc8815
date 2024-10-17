cmake_minimum_required(VERSION 3.22)
project(sc8815)




set(SC8815_SRC sc8815.c)
set(INCLUDE_DIRS .)

add_library(sc8815 STATIC
        ${SC8815_SRC}
        )

target_include_directories(sc8815 PUBLIC ${INCLUDE_DIRS})
target_link_libraries(sc8815)


add_executable(sc8815_test main.c)
target_link_libraries(sc8815_test sc8815)
