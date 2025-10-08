cmake_minimum_required(VERSION 3.10)
project(camera_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- 基础依赖 ----
set(OpenCv_DIR "/home/li/opencv/lib/cmake/opencv4")
set(TensorRT_ROOT "/home/li/TensorRT-8.6.1.6")

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)
find_package(k4a REQUIRED)
find_package(CUDA REQUIRED)

# ---- 检查ROS环境 ----
if(DEFINED ENV{ROS_DISTRO} OR DEFINED CATKIN_DEVEL_PREFIX)
    message(STATUS "🔹 Detected ROS, building with ROS support")
    set(BUILD_WITH_ROS ON)

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
    )

    catkin_package(
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
    )

    include_directories(
        ${catkin_INCLUDE_DIRS}
        /usr/local/cuda/include
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${TensorRT_ROOT}/include
        inc
    )

else()
    message(STATUS "🔸 No ROS detected, building without ROS")
    set(BUILD_WITH_ROS OFF)

    include_directories(
        /usr/local/cuda/include
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${TensorRT_ROOT}/include
        inc
    )
endif()

# ---- 链接路径 ----
link_directories(
    /home/li/TensorRT-8.6.1.6/lib
    /usr/local/cuda/lib64
    /usr/lib/x86_64-linux-gnu
)

# ---- YOLO子模块 ----
add_subdirectory(src/yolo)

#             通用可执行文件

#---realsense---
add_executable(rs_viewer src/rsmain.cpp src/realsense.cpp src/myinfer.cpp src/yolo/yolo.cu)
target_link_libraries(rs_viewer PRIVATE
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    nvinfer
    nvonnxparser
    realsense2::realsense2
    ${CUDA_LIBRARIES}
    yolo
    -lffi
)


#---K4A---
add_executable(k4a_viewer src/k4amain.cpp src/camera_k4a.cpp src/myinfer.cpp src/yolo/yolo.cu)
target_link_libraries(k4a_viewer PRIVATE
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    nvinfer
    nvonnxparser
    k4a::k4a
    ${CUDA_LIBRARIES}
    yolo
    -lffi
)

#---one_eye---
add_executable(one_eye src/one_eye.cpp src/myinfer.cpp src/yolo/yolo.cu src/realsense.cpp)
target_link_libraries(one_eye PRIVATE
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    nvinfer
    nvonnxparser
    realsense2::realsense2
    ${CUDA_LIBRARIES}
    yolo
    -lffi
)

#             ROS可执行文件（仅在ROS下编译）
if(BUILD_WITH_ROS)
    #---k4a_viewer_ros---
    add_executable(k4a_viewer_ros src/k4amain.cpp src/camera_k4a.cpp src/myinfer.cpp src/yolo/yolo.cu)
    target_compile_definitions(k4a_viewer_ros PRIVATE BUILD_WITH_ROS)
    target_include_directories(k4a_viewer_ros PRIVATE
        ${catkin_INCLUDE_DIRS}
        inc
    )
    target_link_libraries(k4a_viewer_ros PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        nvinfer
        nvonnxparser
        k4a::k4a
        ${CUDA_LIBRARIES}
        yolo
        pthread
        -lffi
    )

    install(TARGETS k4a_viewer_ros
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

    #---rs_viewer_ros---
    add_executable(rs_viewer_ros src/rsmain.cpp src/realsense.cpp src/myinfer.cpp src/yolo/yolo.cu)
    target_compile_definitions(rs_viewer_ros PRIVATE BUILD_WITH_ROS)
    target_include_directories(rs_viewer_ros PRIVATE ${catkin_INCLUDE_DIRS} inc)
    target_link_libraries(rs_viewer_ros PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        nvinfer
        nvonnxparser
        realsense2::realsense2
        ${CUDA_LIBRARIES}
        yolo
        pthread
        -lffi
    )
    install(TARGETS rs_viewer_ros RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

    #---one_eye_ros---
    add_executable(one_eye_ros src/one_eye.cpp src/myinfer.cpp src/yolo/yolo.cu src/realsense.cpp)
    target_compile_definitions(one_eye_ros PRIVATE BUILD_WITH_ROS)
    target_include_directories(one_eye_ros PRIVATE ${catkin_INCLUDE_DIRS} inc)
    target_link_libraries(one_eye_ros PRIVATE
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        nvinfer
        nvonnxparser
        realsense2::realsense2
        ${CUDA_LIBRARIES}
        yolo
        pthread
        -lffi
    )
    install(TARGETS one_eye_ros RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

endif()
