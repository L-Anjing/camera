cmake_minimum_required(VERSION 3.10)
project(camera_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TensorRT_ROOT "$ENV{HOME}/TensorRT-8.6.1.6" CACHE PATH "TensorRT install root")
set(PCL_DIR "/opt/pcl-1.15/lib/cmake/pcl") 
# 如果按照教程可以试试先注释此行，找不到在手动指定,此处源码编译pcl时使用cmake -DCMAKE_INSTALL_PREFIX=/opt/pcl-1.15 ..
# ---- 基础依赖 ----
find_package(realsense2 REQUIRED)
find_package(PCL 1.15 EXACT REQUIRED)
find_package(k4a REQUIRED)
find_package(CUDA REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# ---- ROS模式选择 ----
set(ROS_VERSION "AUTO" CACHE STRING "ROS mode: AUTO, NONE, ROS1, ROS2")
set_property(CACHE ROS_VERSION PROPERTY STRINGS AUTO NONE ROS1 ROS2)

if(ROS_VERSION STREQUAL "AUTO")
    if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
        set(ROS_VERSION "ROS1")
    elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
        set(ROS_VERSION "ROS2")
    elseif(DEFINED CATKIN_DEVEL_PREFIX)
        set(ROS_VERSION "ROS1")
    else()
        set(ROS_VERSION "NONE")
    endif()
endif()

set(BUILD_WITH_ROS OFF)
set(BUILD_WITH_ROS1 OFF)
set(BUILD_WITH_ROS2 OFF)

if(ROS_VERSION STREQUAL "ROS1")
    message(STATUS "🔹 Building with ROS1 support")
    set(BUILD_WITH_ROS ON)
    set(BUILD_WITH_ROS1 ON)

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        serial
    )

    catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
    )
elseif(ROS_VERSION STREQUAL "ROS2")
    message(STATUS "🔹 Building with ROS2 support")
    set(BUILD_WITH_ROS ON)
    set(BUILD_WITH_ROS2 ON)

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    find_path(SERIAL_INCLUDE_DIR serial/serial.h)
    find_library(SERIAL_LIBRARY serial)
    if(NOT SERIAL_INCLUDE_DIR OR NOT SERIAL_LIBRARY)
        message(FATAL_ERROR "ROS2 serial node requires serial library (serial/serial.h, libserial).")
    endif()
elseif(ROS_VERSION STREQUAL "NONE")
    message(STATUS "🔸 Building without ROS support")
else()
    message(FATAL_ERROR "Invalid ROS_VERSION='${ROS_VERSION}'. Use AUTO/NONE/ROS1/ROS2")
endif()

# ---- 包含路径 ----
include_directories(
    ${PROJECT_SOURCE_DIR}/inc
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    /usr/local/cuda/include
    ${TENSORRT_INCLUDE_DIR}
)

if(BUILD_WITH_ROS1)
    include_directories(${catkin_INCLUDE_DIRS})
endif()


# ---- 链接路径 ----
link_directories(
    ${TENSORRT_LIB_DIR}
    /usr/local/cuda/lib64
    /usr/lib/x86_64-linux-gnu
)

# ---- YOLO子模块 ----
add_subdirectory(src/yolo)

# ---- 通用库定义 ----
set(COMMON_LIBS
    ${PCL_LIBRARIES}
    ${TENSORRT_NVINFER_LIB}
    ${TENSORRT_NVONNXPARSER_LIB}
    ${CUDA_LIBRARIES}
    yolo
    yaml-cpp
    ffi
    ${OpenCV_LIBS}
)
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")

# ---- 通用源文件 ----
set(COMMON_SOURCES
    src/common/camera_params.cpp
    src/utils/vision_draw.cpp
)

# ---- RealSense 源文件 ----
set(RS_SOURCES
    ${COMMON_SOURCES}
    src/realsense/rs_detect.cpp
    src/realsense/realsense.cpp
    src/utils/myinfer.cpp
    src/yolo/yolo.cu
)

# ---- K4A 源文件 ----
set(K4A_BASE_SOURCES
    ${COMMON_SOURCES}
    src/k4a/camera_k4a.cpp
    src/utils/myinfer.cpp
    src/utils/block_recognizer.cpp
    src/yolo/yolo.cu
)

set(K4A_DETECT_SOURCES
    ${K4A_BASE_SOURCES}
    src/k4a/k4a_detect.cpp
)

set(K4A_RECORD_SOURCES
    ${COMMON_SOURCES}
    src/k4a/k4a_capture_videos.cpp
    src/k4a/camera_k4a.cpp
)

set(K4A_CAPTURE_SOURCES
    ${COMMON_SOURCES}
    src/k4a/k4a_capture_images.cpp
    src/k4a/camera_k4a.cpp
)
#---RealSense---
add_executable(rs_viewer ${RS_SOURCES})
target_link_libraries(rs_viewer PRIVATE
    realsense2::realsense2
    ${COMMON_LIBS}
)
target_compile_options(rs_viewer PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)

#---K4A Detect---
add_executable(k4a_detect ${K4A_DETECT_SOURCES})
target_link_libraries(k4a_detect PRIVATE
    k4a::k4a
    ${COMMON_LIBS}
)
target_compile_options(k4a_detect PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)

#---K4A Record Videos---
add_executable(record_videos ${K4A_RECORD_SOURCES})
target_link_libraries(record_videos PRIVATE
    k4a::k4a
    ${COMMON_LIBS}
)

#---K4A Capture Images---
add_executable(capture_images ${K4A_CAPTURE_SOURCES})
target_link_libraries(capture_images PRIVATE
    k4a::k4a
    ${COMMON_LIBS}
)



#             ROS可执行文件（仅在ROS下编译）
if(BUILD_WITH_ROS)
    # ---- ROS编译定义 ----
    set(ROS_LIBS
        ${COMMON_LIBS}
        pthread
    )

    if(BUILD_WITH_ROS1)
        list(APPEND ROS_LIBS ${catkin_LIBRARIES})
    endif()

    #---K4A Detect ROS---
    add_executable(k4a_detect_ros ${K4A_DETECT_SOURCES})
    target_compile_definitions(k4a_detect_ros PRIVATE BUILD_WITH_ROS)
    if(BUILD_WITH_ROS1)
        target_compile_definitions(k4a_detect_ros PRIVATE BUILD_WITH_ROS1)
        target_include_directories(k4a_detect_ros PRIVATE ${catkin_INCLUDE_DIRS})
    elseif(BUILD_WITH_ROS2)
        target_compile_definitions(k4a_detect_ros PRIVATE BUILD_WITH_ROS2)
        ament_target_dependencies(k4a_detect_ros rclcpp std_msgs)
    endif()
    target_link_libraries(k4a_detect_ros PRIVATE
        k4a::k4a
        ${ROS_LIBS}
    )
    target_compile_options(k4a_detect_ros PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
    if(BUILD_WITH_ROS1)
        install(TARGETS k4a_detect_ros
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
    else()
        install(TARGETS k4a_detect_ros
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()

    #---RealSense ROS---
    add_executable(rs_viewer_ros ${RS_SOURCES})
    target_compile_definitions(rs_viewer_ros PRIVATE BUILD_WITH_ROS)
    if(BUILD_WITH_ROS1)
        target_compile_definitions(rs_viewer_ros PRIVATE BUILD_WITH_ROS1)
        target_include_directories(rs_viewer_ros PRIVATE ${catkin_INCLUDE_DIRS})
    elseif(BUILD_WITH_ROS2)
        target_compile_definitions(rs_viewer_ros PRIVATE BUILD_WITH_ROS2)
        ament_target_dependencies(rs_viewer_ros rclcpp std_msgs)
    endif()
    target_link_libraries(rs_viewer_ros PRIVATE
        realsense2::realsense2
        ${ROS_LIBS}
    )
    target_compile_options(rs_viewer_ros PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
    if(BUILD_WITH_ROS1)
        install(TARGETS rs_viewer_ros
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
    else()
        install(TARGETS rs_viewer_ros
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()

    #---Serial Bridge ROS---
    add_executable(k4a_serial_node src/utils/serial.cpp)
    target_compile_definitions(k4a_serial_node PRIVATE BUILD_WITH_ROS)
    if(BUILD_WITH_ROS1)
        target_compile_definitions(k4a_serial_node PRIVATE BUILD_WITH_ROS1)
        target_include_directories(k4a_serial_node PRIVATE ${catkin_INCLUDE_DIRS})
        target_link_libraries(k4a_serial_node PRIVATE ${catkin_LIBRARIES} pthread)
        install(TARGETS k4a_serial_node
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
    elseif(BUILD_WITH_ROS2)
        target_compile_definitions(k4a_serial_node PRIVATE BUILD_WITH_ROS2)
        target_include_directories(k4a_serial_node PRIVATE ${SERIAL_INCLUDE_DIR})
        target_link_libraries(k4a_serial_node PRIVATE ${SERIAL_LIBRARY} pthread)
        ament_target_dependencies(k4a_serial_node rclcpp std_msgs)
        install(TARGETS k4a_serial_node
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()
endif()

if(BUILD_WITH_ROS2)
    install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch.py"
    )
    ament_package()
endif()
