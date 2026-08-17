cmake_minimum_required(VERSION 3.18)
project(rm_buff_tracker LANGUAGES CXX)

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
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
    add_compile_options(/utf-8)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio dnn calib3d)

# === Optional ONNX Runtime ===
if(NOT DEFINED ONNXRUNTIME_DIR)
    if(DEFINED ENV{ONNXRUNTIME_DIR} AND NOT "$ENV{ONNXRUNTIME_DIR}" STREQUAL "")
        set(ONNXRUNTIME_DIR "$ENV{ONNXRUNTIME_DIR}" CACHE PATH "ONNX Runtime directory")
    elseif(DEFINED DEFAULT_ONNXRUNTIME_DIR AND NOT "${DEFAULT_ONNXRUNTIME_DIR}" STREQUAL "")
        set(ONNXRUNTIME_DIR "${DEFAULT_ONNXRUNTIME_DIR}" CACHE PATH "ONNX Runtime directory")
    endif()
endif()

set(HAVE_ONNXRUNTIME OFF)
if(DEFINED ONNXRUNTIME_DIR AND EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h")
    set(HAVE_ONNXRUNTIME ON)
    message(STATUS "ONNX Runtime found at ${ONNXRUNTIME_DIR}")
else()
    message(STATUS "ONNX Runtime not found, YOLO will use OpenCV DNN fallback")
endif()

# === Core library (no GUI, no ROS) ===
add_library(buff_core STATIC
    src/core/parameter.cpp
    src/core/buff_tracker.cpp
    src/core/angle_processor.cpp
    src/core/hsv_detector.cpp
    src/core/yolo_detector.cpp
    src/core/compensation.cpp
    src/core/buff_pipeline.cpp
    src/core/camera_geometry.cpp
)
target_include_directories(buff_core PUBLIC src src/core ${OpenCV_INCLUDE_DIRS})
target_link_libraries(buff_core PUBLIC ${OpenCV_LIBS})

if(HAVE_ONNXRUNTIME)
    target_include_directories(buff_core PUBLIC "${ONNXRUNTIME_DIR}/include")
    target_link_libraries(buff_core PUBLIC "${ONNXRUNTIME_DIR}/lib/onnxruntime.lib")
    target_compile_definitions(buff_core PUBLIC HAVE_ONNXRUNTIME=1)
endif()

# === Standalone executable (always built) ===
add_executable(predict_example_main
    src/standalone/standalone_main.cpp
    src/standalone/standalone_options.cpp
    src/standalone/standalone_visual_tools.cpp
)
target_link_libraries(predict_example_main PRIVATE buff_core)

add_executable(yolo_video_test
    tests/yolo_video_test.cpp
)
target_link_libraries(yolo_video_test PRIVATE buff_core)

add_executable(yolo_image_test
    tests/yolo_image_test.cpp
)
target_link_libraries(yolo_image_test PRIVATE buff_core)

add_executable(camera_geometry_test
    tests/camera_geometry_test.cpp
)
target_link_libraries(camera_geometry_test PRIVATE buff_core)

include(CTest)
if(BUILD_TESTING)
    add_test(NAME camera_geometry_test COMMAND camera_geometry_test)
endif()

if(WIN32 AND HAVE_ONNXRUNTIME)
    set(ONNXRUNTIME_DLL "${ONNXRUNTIME_DIR}/lib/onnxruntime.dll")
    set(ONNXRUNTIME_PROVIDERS_SHARED_DLL "${ONNXRUNTIME_DIR}/lib/onnxruntime_providers_shared.dll")

    if(EXISTS "${ONNXRUNTIME_DLL}")
        foreach(runtime_target predict_example_main yolo_video_test yolo_image_test camera_geometry_test)
            add_custom_command(TARGET ${runtime_target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "${ONNXRUNTIME_DLL}"
                        "$<TARGET_FILE_DIR:${runtime_target}>")
            if(EXISTS "${ONNXRUNTIME_PROVIDERS_SHARED_DLL}")
                add_custom_command(TARGET ${runtime_target} POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                            "${ONNXRUNTIME_PROVIDERS_SHARED_DLL}"
                            "$<TARGET_FILE_DIR:${runtime_target}>")
            endif()
        endforeach()
    else()
        message(WARNING "ONNX Runtime import library was found, but runtime DLL is missing: ${ONNXRUNTIME_DLL}")
    endif()
endif()

# === Optional ROS 2 build ===
find_package(ament_cmake QUIET)
find_package(ament_index_cpp QUIET)
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(std_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(tf2 QUIET)
find_package(tf2_ros QUIET)
find_package(tf2_geometry_msgs QUIET)
find_package(rosidl_default_generators QUIET)

if(ament_cmake_FOUND AND ament_index_cpp_FOUND AND rclcpp_FOUND AND sensor_msgs_FOUND AND
   std_msgs_FOUND AND geometry_msgs_FOUND AND rosidl_default_generators_FOUND)
    message(STATUS "ROS 2 environment found, building buff_node")

    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/BuffTarget.msg"
        "msg/BuffObservation.msg"
        "msg/BuffTargetState.msg"
        DEPENDENCIES std_msgs geometry_msgs
    )

    add_executable(buff_node
        src/ros2/buff_node.cpp
        src/ros2/buff_ros_utils.cpp
        src/ros2/ros2_main.cpp
    )
    target_link_libraries(buff_node buff_core)
    ament_target_dependencies(buff_node
        ament_index_cpp rclcpp sensor_msgs std_msgs geometry_msgs)
    rosidl_get_typesupport_target(cpp_typesupport_target
        ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(buff_node "${cpp_typesupport_target}")

    add_executable(topic_probe
        src/ros2/topic_probe.cpp
    )
    ament_target_dependencies(topic_probe
        rclcpp sensor_msgs std_msgs geometry_msgs)

    set(RM_BUFF_ROS_TARGETS buff_node topic_probe predict_example_main)

    if(tf2_FOUND AND tf2_ros_FOUND AND tf2_geometry_msgs_FOUND)
        message(STATUS "tf2 found, building independent BUFF spatial channel")

        add_executable(buff_detector_node
            src/ros2/buff_detector_node.cpp
            src/ros2/buff_ros_utils.cpp
            src/ros2/buff_spatial_utils.cpp
        )
        target_link_libraries(buff_detector_node buff_core "${cpp_typesupport_target}")
        ament_target_dependencies(buff_detector_node
            ament_index_cpp rclcpp sensor_msgs std_msgs geometry_msgs tf2 tf2_geometry_msgs)

        add_executable(buff_tracker_node
            src/ros2/buff_tracker_node.cpp
        )
        target_link_libraries(buff_tracker_node buff_core "${cpp_typesupport_target}")
        ament_target_dependencies(buff_tracker_node
            rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

        list(APPEND RM_BUFF_ROS_TARGETS buff_detector_node buff_tracker_node)
    else()
        message(STATUS "tf2 not found, skipping independent BUFF spatial channel")
    endif()

    install(TARGETS ${RM_BUFF_ROS_TARGETS}
            DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY config/
            DESTINATION share/${PROJECT_NAME}/config)
    install(DIRECTORY launch/
            DESTINATION share/${PROJECT_NAME}/launch)
    install(DIRECTORY models/
            DESTINATION share/${PROJECT_NAME}/models)

    if(WIN32 AND HAVE_ONNXRUNTIME)
        file(TO_CMAKE_PATH "${ONNXRUNTIME_DIR}/lib/onnxruntime.dll" ONNXRUNTIME_DLL)
        file(TO_CMAKE_PATH "${ONNXRUNTIME_DIR}/lib/onnxruntime_providers_shared.dll" ONNXRUNTIME_PROVIDERS_SHARED_DLL)

        install(FILES "${ONNXRUNTIME_DLL}"
                DESTINATION lib/${PROJECT_NAME})
        if(EXISTS "${ONNXRUNTIME_PROVIDERS_SHARED_DLL}")
            install(FILES "${ONNXRUNTIME_PROVIDERS_SHARED_DLL}"
                    DESTINATION lib/${PROJECT_NAME})
        endif()
    endif()

    ament_package()
else()
    message(STATUS "ROS 2 not found, building standalone only")
endif()
