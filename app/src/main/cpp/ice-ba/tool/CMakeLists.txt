cmake_minimum_required(VERSION 2.8.11)
project(TOOL)
if (CMAKE_SYSTEM_NAME MATCHES "Linux")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../bin)
endif()

include_directories(${OpenCV_INCLUDE_DIR})

add_executable(calibr_fisheye
calibr_fisheye.cpp
)
target_link_libraries(calibr_fisheye
    ${OpenCV_LIBS}
)
