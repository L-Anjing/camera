cmake_minimum_required(VERSION 3.10)
project(camera_bridge)

# ---- 基础环境配置 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 手动指定 TensorRT 路径 (基于你之前的安装路径)
set(TensorRT_ROOT "/opt/TensorRT-8.6.1.6")
set(TENSORRT_INCLUDE_DIR "${TensorRT_ROOT}/include")
set(TENSORRT_LIB_DIR "${TensorRT_ROOT}/lib")

# ---- ROS2 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# ---- 其他依赖 ----
find_package(CUDA REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)
find_package(k4a REQUIRED)
find_package(OrbbecSDK QUIET)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# TensorRT 库查找
find_library(TENSORRT_NVINFER_LIB nvinfer HINTS ${TENSORRT_LIB_DIR})
find_library(TENSORRT_NVONNXPARSER_LIB nvonnxparser HINTS ${TENSORRT_LIB_DIR})

# Serial 库
find_path(SERIAL_INCLUDE_DIR serial/serial.h
    HINTS /home/pi/workspace/3rd_ws/install/serial/include)
find_library(SERIAL_LIBRARY serial
    HINTS /home/pi/workspace/3rd_ws/install/serial/lib)
if(NOT SERIAL_INCLUDE_DIR OR NOT SERIAL_LIBRARY)
    message(FATAL_ERROR "serial node requires serial/serial.h and libserial.so")
endif()
get_filename_component(SERIAL_LIBRARY_DIR "${SERIAL_LIBRARY}" DIRECTORY)

# ---- 包含与链接路径 ----
include_directories(
    inc
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIR}
)

link_directories(
    ${TENSORRT_LIB_DIR}
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
)

# ---- YOLO 子模块 ----
add_subdirectory(src/utils/yolo)

# ---- 通用库定义 ----
set(COMMON_LIBS
    ${PCL_LIBRARIES}
    ${TENSORRT_NVINFER_LIB}
    ${TENSORRT_NVONNXPARSER_LIB}
    ${CUDA_LIBRARIES}
    yolo
    yaml-cpp
    ${OpenCV_LIBS}
    pthread
)

# ---- 源文件定义 ----
set(COMMON_SOURCES src/common/camera_params.cpp src/utils/vision_draw.cpp src/utils/kalman_tracker.cpp)
set(RS_SOURCES ${COMMON_SOURCES} src/realsense/rs_detect.cpp src/realsense/realsense.cpp src/utils/myinfer.cpp src/utils/yolo/yolo.cu)
set(K4A_DETECT_SOURCES ${COMMON_SOURCES} src/k4a/camera_k4a.cpp src/usbcam/usbcam.cpp src/utils/myinfer.cpp src/utils/block_recognizer.cpp src/k4a/k4a_detect.cpp src/utils/yolo/yolo.cu)
set(ORBBEC_DETECT_SOURCES ${COMMON_SOURCES} src/orbbec/camera_orbbec.cpp src/usbcam/usbcam.cpp src/utils/myinfer.cpp src/utils/block_recognizer.cpp src/orbbec/orbbec_detect.cpp src/utils/yolo/yolo.cu)
set(INSIGHT9_DETECT_SOURCES ${COMMON_SOURCES} src/insight9/camera_insight9.cpp src/usbcam/usbcam.cpp src/utils/myinfer.cpp src/utils/block_recognizer.cpp src/insight9/insight9_detect.cpp src/utils/yolo/yolo.cu)
set(SERIAL_NODE_SOURCES src/utils/serial.cpp)

# ---- K4A Detect Node ----
add_executable(k4a_detect ${K4A_DETECT_SOURCES})
target_compile_options(k4a_detect PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
ament_target_dependencies(k4a_detect rclcpp std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(k4a_detect k4a::k4a ${COMMON_LIBS})

# ---- Orbbec Femto Bolt Detect Node ----
if(OrbbecSDK_FOUND)
    if(TARGET OrbbecSDK::OrbbecSDK)
        set(ORBBECSDK_LIB OrbbecSDK::OrbbecSDK)
    elseif(DEFINED OrbbecSDK_LIBRARIES)
        set(ORBBECSDK_LIB ${OrbbecSDK_LIBRARIES})
    elseif(DEFINED OrbbecSDK_LIBRARY)
        set(ORBBECSDK_LIB ${OrbbecSDK_LIBRARY})
    else()
        set(ORBBECSDK_LIB OrbbecSDK)
    endif()

    add_executable(orbbec_detect ${ORBBEC_DETECT_SOURCES})
    target_compile_options(orbbec_detect PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
    if(DEFINED OrbbecSDK_INCLUDE_DIRS)
        target_include_directories(orbbec_detect PRIVATE ${OrbbecSDK_INCLUDE_DIRS})
    elseif(DEFINED OrbbecSDK_INCLUDE_DIR)
        target_include_directories(orbbec_detect PRIVATE ${OrbbecSDK_INCLUDE_DIR})
    endif()
    ament_target_dependencies(orbbec_detect rclcpp std_msgs sensor_msgs cv_bridge image_transport)
    target_link_libraries(orbbec_detect ${ORBBECSDK_LIB} ${COMMON_LIBS})

    add_executable(orbbec_capture_images
        src/orbbec/orbbec_capture_images.cpp
        src/orbbec/camera_orbbec.cpp
        ${COMMON_SOURCES}
    )
    target_compile_options(orbbec_capture_images PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
    if(DEFINED OrbbecSDK_INCLUDE_DIRS)
        target_include_directories(orbbec_capture_images PRIVATE ${OrbbecSDK_INCLUDE_DIRS})
    elseif(DEFINED OrbbecSDK_INCLUDE_DIR)
        target_include_directories(orbbec_capture_images PRIVATE ${OrbbecSDK_INCLUDE_DIR})
    endif()
    ament_target_dependencies(orbbec_capture_images
        rclcpp std_msgs sensor_msgs cv_bridge image_transport)
    target_link_libraries(orbbec_capture_images ${ORBBECSDK_LIB} ${COMMON_LIBS})

    add_executable(orbbec_capture_videos
        src/orbbec/orbbec_capture_videos.cpp
        src/orbbec/camera_orbbec.cpp
        ${COMMON_SOURCES}
    )
    target_compile_options(orbbec_capture_videos PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
    if(DEFINED OrbbecSDK_INCLUDE_DIRS)
        target_include_directories(orbbec_capture_videos PRIVATE ${OrbbecSDK_INCLUDE_DIRS})
    elseif(DEFINED OrbbecSDK_INCLUDE_DIR)
        target_include_directories(orbbec_capture_videos PRIVATE ${OrbbecSDK_INCLUDE_DIR})
    endif()
    ament_target_dependencies(orbbec_capture_videos
        rclcpp std_msgs sensor_msgs cv_bridge image_transport)
    target_link_libraries(orbbec_capture_videos ${ORBBECSDK_LIB} ${COMMON_LIBS})
else()
    message(WARNING "OrbbecSDK not found; skip Orbbec Femto Bolt targets.")
endif()

# ---- Insight9 Detect Node ----
add_executable(insight9_detect ${INSIGHT9_DETECT_SOURCES})
target_compile_options(insight9_detect PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
ament_target_dependencies(insight9_detect rclcpp std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(insight9_detect ${COMMON_LIBS})

# ---- USB Camera Detect Node ----
set(USB_DETECT_SOURCES ${COMMON_SOURCES} src/usbcam/usbcam.cpp src/utils/myinfer.cpp src/utils/block_recognizer.cpp src/usbcam/usb_detect.cpp src/utils/yolo/yolo.cu)
add_executable(usb_detect ${USB_DETECT_SOURCES})
target_compile_options(usb_detect PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
ament_target_dependencies(usb_detect rclcpp std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(usb_detect ${COMMON_LIBS})

# ---- K4A Capture Images Node ----
add_executable(k4a_capture_images
    src/k4a/k4a_capture_images.cpp
    src/k4a/camera_k4a.cpp
    ${COMMON_SOURCES}
)
target_compile_options(k4a_capture_images PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
ament_target_dependencies(k4a_capture_images
    rclcpp std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(k4a_capture_images k4a::k4a ${COMMON_LIBS})

# ---- RealSense Viewer Node ----
add_executable(rs_viewer ${RS_SOURCES})
target_compile_options(rs_viewer PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
ament_target_dependencies(rs_viewer rclcpp std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(rs_viewer realsense2::realsense2 ${COMMON_LIBS})

# ---- Serial Node ----
add_executable(serial_node ${SERIAL_NODE_SOURCES})
ament_target_dependencies(serial_node rclcpp std_msgs)
target_include_directories(serial_node PRIVATE ${SERIAL_INCLUDE_DIR})
target_link_libraries(serial_node ${SERIAL_LIBRARY})
set_target_properties(serial_node PROPERTIES
    BUILD_RPATH "${SERIAL_LIBRARY_DIR}"
    INSTALL_RPATH "${SERIAL_LIBRARY_DIR}")

# ---- 安装 ----
install(TARGETS
    k4a_detect
    insight9_detect
    usb_detect
    k4a_capture_images
    rs_viewer
    serial_node
    DESTINATION lib/${PROJECT_NAME})

if(OrbbecSDK_FOUND)
    install(TARGETS
        orbbec_detect
        orbbec_capture_images
        orbbec_capture_videos
        DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

# ---- Ament 导出 ----
ament_export_dependencies(rclcpp std_msgs sensor_msgs cv_bridge image_transport)
ament_export_include_directories(inc)
ament_export_libraries(yolo)

ament_package()
