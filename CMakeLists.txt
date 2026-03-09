cmake_minimum_required(VERSION 3.18)
project(RM_Buff_Tracker_GUT_cpp LANGUAGES CXX)

# OpenCV_DIR resolution priority:
# 1) -DOpenCV_DIR=... (manual)
# 2) environment OpenCV_DIR
# 3) DEFAULT_OPENCV_DIR from CMake preset
if(NOT DEFINED OpenCV_DIR)
    if(DEFINED ENV{OpenCV_DIR} AND NOT "$ENV{OpenCV_DIR}" STREQUAL "")
        set(OpenCV_DIR "$ENV{OpenCV_DIR}" CACHE PATH "OpenCV config directory")
    elseif(DEFINED DEFAULT_OPENCV_DIR AND NOT "${DEFAULT_OPENCV_DIR}" STREQUAL "")
        set(OpenCV_DIR "${DEFAULT_OPENCV_DIR}" CACHE PATH "OpenCV config directory")
    endif()
endif()

if(MSVC AND DEFINED OpenCV_DIR)
    string(TOLOWER "${OpenCV_DIR}" OPENCV_DIR_LOWER_PRECHECK)
    if(OPENCV_DIR_LOWER_PRECHECK MATCHES "msys64[/\\\\]mingw")
        if(DEFINED DEFAULT_OPENCV_DIR AND NOT "${DEFAULT_OPENCV_DIR}" STREQUAL "")
            message(WARNING "OpenCV_DIR from cache/environment points to MinGW OpenCV under MSVC. Auto-reset to DEFAULT_OPENCV_DIR.")
            set(OpenCV_DIR "${DEFAULT_OPENCV_DIR}" CACHE PATH "OpenCV config directory" FORCE)
        else()
            message(FATAL_ERROR
                "MSVC toolchain detected, but OpenCV_DIR points to MinGW/OpenCV (msys64). "
                "Please use OpenCV_DIR=D:/develop/opencv_windows/opencv/build")
        endif()
    endif()
endif()

if(MSVC AND DEFINED OpenCV_DIR AND NOT "${OpenCV_DIR}" STREQUAL "")
    set(OPENCV_VC15_CANDIDATE "${OpenCV_DIR}/x64/vc15/lib")
    if(EXISTS "${OPENCV_VC15_CANDIDATE}/OpenCVConfig.cmake")
        message(STATUS "MSVC detected: using OpenCV config fallback at ${OPENCV_VC15_CANDIDATE}")
        set(OpenCV_DIR "${OPENCV_VC15_CANDIDATE}" CACHE PATH "OpenCV config directory" FORCE)
    endif()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio)

add_executable(predict_example_main
    src/predict_example_main.cpp
    src/parameter.cpp
    src/buff_tracker.cpp
    src/angle_processor.cpp
)

target_include_directories(predict_example_main PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(predict_example_main PRIVATE ${OpenCV_LIBS})
