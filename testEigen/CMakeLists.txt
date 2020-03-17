cmake_minimum_required(VERSION 3.10)
project(testEigen)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include ./include)
add_executable(testEigen main.cpp)
target_link_libraries(testEigen ${OpenCV_LIBRARIES})
#target_compile_options(testEigen PUBLIC -Wall -Wextra -pedantic)
