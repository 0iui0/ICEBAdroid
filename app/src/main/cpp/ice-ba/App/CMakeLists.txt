cmake_minimum_required(VERSION 2.8.11)
project(EuRoC_test)
if (CMAKE_SYSTEM_NAME MATCHES "Linux")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/../bin)

#这样使用暂时不对，原因不明
# OPTION(SLAM_TEST "buil to test when load data speed higher than process" OFF)
# IF(SLAM_TEST)
#   add_definitions("-DSLAM_TEST")
#   message("buil to test when load data speed higher than process ----------------------------------------------------------------")
# endif(SLAM_TEST)

# =========================
# run front-end + back-end with EuRoC dataset
# =========================
add_executable(ice_ba
    EuRoC_test.cpp
)
target_link_libraries(ice_ba
    IBA
    OF
)

#ROS
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
geometry_msgs
nav_msgs
tf
cv_bridge
)
include_directories(
${catkin_INCLUDE_DIRS}
)
add_executable(ice_ba_ros
    ICE-BA-ROS.cpp
)
target_link_libraries(ice_ba_ros
    IBA
    OF
    ${catkin_LIBRARIES}
)

if (CVD_FOUND)
 target_link_libraries(ice_ba
     IBAVis
)
endif(CVD_FOUND)



# =========================
# run back-end only with .dat file
# =========================
add_executable(back_end
    ../App/run_backend.cpp
)
target_include_directories(back_end PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GLOBAL_IBA_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
)
target_link_libraries(back_end
    IBA
    OF
)
if (CVD_FOUND)
 target_include_directories(back_end PUBLIC
     ${GLOBAL_VISUALIZATION_INCLUDE_DIR}
 )
 target_link_libraries(back_end
     IBAVis
 )
endif (CVD_FOUND)

 #写来给安卓调用
else()

# add_library(
#    EuRoC_test
#    SHARED
#    ../App/EuRoC_test.cpp
# )
# target_link_libraries(
#    EuRoC_test
#    IBA
#    OF
# )

endif()