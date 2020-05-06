cmake_minimum_required(VERSION 3.10)
project(reconstruction)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include ./include)
add_executable(reconstruction main.cpp mesh_io.cpp mesh_io.h)
target_link_libraries(reconstruction ${OpenCV_LIBRARIES})
#target_compile_options(reconstruction PUBLIC -Wall -Wextra -pedantic)
