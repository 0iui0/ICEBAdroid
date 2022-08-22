cmake_minimum_required(VERSION 3.4.1)

#以后都这样用，这样就不用在判断平台啥的
set(LIB_NAME NativeLib)
set(
        ${LIB_NAME}_SRCS
        nativelib.cpp
        ice-ba/App/EuRoC_test.cpp
)
set(OpenCV_DIR /home/an/OpenCV-android-sdk/sdk/native/jni)
find_package(OpenCV REQUIRED)

if (OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    message(STATUS "OpenCV library status:")
    message(STATUS "    version:${OpenCV_VERSION}")
    message(STATUS "    libraries:${OpenCV_LIBS}")
    message(STATUS "    include path:${OpenCV_INCLUDE_DIRS}")
else (OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV library not found")
endif (OpenCV_FOUND)


#这里使用的是htts://github.com/moritz-wundke/Boost-for-Android
set(ThirdParty ${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty)
set(Boost_INCLUDE_DIR ${ThirdParty}/include/boost-1_74)
set(Boost_LIBRARY_DIR ${ThirdParty}/libs/${ANDROID_ABI})
#用通配符同时添加多个lib库文件依赖
file(GLOB Boost_LIBRARIES "${Boost_LIBRARY_DIR}/libboost*.a")
include_directories(${Boost_INCLUDE_DIR})

set(glog_binary_dir ${ThirdParty}/libs/${ANDROID_ABI}/libglog.a)
add_library(
        glog
        STATIC
        IMPORTED
        GLOBAL
)
set_target_properties(
        glog
        PROPERTIES IMPORTED_LOCATION
        ${glog_binary_dir}
)

set(yamlcpp_binary_dir ${ThirdParty}/libs/${ANDROID_ABI}/libyaml-cpp.a)
set(YAMLCPP_LIBRARIES YAMLCPP_LIBRARIES)
add_library(
        ${YAMLCPP_LIBRARIES}
        STATIC
        IMPORTED
        GLOBAL
)
set_target_properties(
        YAMLCPP_LIBRARIES
        PROPERTIES IMPORTED_LOCATION
        ${yamlcpp_binary_dir}
)


#set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")
if (DEFINED ANDROID_NDK_MAJOR AND ${ANDROID_NDK_MAJOR} GREATER 20)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -static-openmp")
endif ()

include_directories(
        .
        ${OpenCV_INCLUDE_DIRS}
        ${ThirdParty}/include
        ${ThirdParty}/include/eigen3
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(ice-ba)

add_library(
        ${LIB_NAME}
        SHARED
        ${${LIB_NAME}_SRCS}
)
target_link_libraries(
        ${LIB_NAME}
        ${OpenCV_LIBS}
        IBA
        OF
)


find_library(
        log-lib
        log
)


target_link_libraries(
        # Specifies the target library.
        ${LIB_NAME}
        log
        android
)
