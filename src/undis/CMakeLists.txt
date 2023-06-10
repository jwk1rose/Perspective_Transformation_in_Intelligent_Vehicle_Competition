cmake_minimum_required(VERSION 3.23)
project(undis)
set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV  REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(D:/soft/eigen-3.2.10)

add_executable(undis main.cpp undis.h undis.cpp)
target_link_libraries(undis ${OpenCV_LIBS})

