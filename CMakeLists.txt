cmake_minimum_required(VERSION 3.10)
project(camera_bridge)

# ---- 基础环境配置 ----
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 手动指定 TensorRT 路径 (基于你之前的安装路径)
set(TensorRT_ROOT "/opt/TensorRT-8.6.1.6")
set(TENSORRT_INCLUDE_DIR "${TensorRT_ROOT}/include")
set(TENSORRT_LIB_DIR "${TensorRT_ROOT}/lib")

# ---- 寻找依赖 ----
find_package(CUDA REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)
find_package(k4a REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# TensorRT 库查找
find_library(TENSORRT_NVINFER_LIB nvinfer HINTS ${TENSORRT_LIB_DIR})
find_library(TENSORRT_NVONNXPARSER_LIB nvonnxparser HINTS ${TENSORRT_LIB_DIR})

# ---- ROS 模式逻辑 ----
set(ROS_VERSION "AUTO" CACHE STRING "ROS mode: AUTO, NONE, ROS1, ROS2")
if(ROS_VERSION STREQUAL "AUTO")
    if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
        set(ROS_VERSION "ROS2")
    elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
        set(ROS_VERSION "ROS1")
    else()
        set(ROS_VERSION "NONE")
    endif()
endif()

# 初始化标志位
set(BUILD_WITH_ROS OFF)
set(BUILD_WITH_ROS1 OFF)
set(BUILD_WITH_ROS2 OFF)

if(ROS_VERSION STREQUAL "ROS2")
    message(STATUS "🔹 Building with ROS2 (Ament)")
    set(BUILD_WITH_ROS ON)
    set(BUILD_WITH_ROS2 ON)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(image_transport REQUIRED)
    
    # Serial 库处理
    find_path(SERIAL_INCLUDE_DIR serial/serial.h
        HINTS /home/pi/workspace/3rd_ws/install/serial/include)
    find_library(SERIAL_LIBRARY serial
        HINTS /home/pi/workspace/3rd_ws/install/serial/lib)
    if(NOT SERIAL_INCLUDE_DIR OR NOT SERIAL_LIBRARY)
        message(FATAL_ERROR "ROS2 serial node requires serial/serial.h and libserial.so")
    endif()
    get_filename_component(SERIAL_LIBRARY_DIR "${SERIAL_LIBRARY}" DIRECTORY)
elseif(ROS_VERSION STREQUAL "ROS1")
    message(STATUS "🔹 Building with ROS1 (Catkin)")
    set(BUILD_WITH_ROS ON)
    set(BUILD_WITH_ROS1 ON)
    find_package(catkin REQUIRED COMPONENTS 
        roscpp std_msgs sensor_msgs cv_bridge image_transport serial)
endif()

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
add_subdirectory(src/yolo)

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
set(COMMON_SOURCES src/common/camera_params.cpp src/utils/vision_draw.cpp)
set(RS_SOURCES ${COMMON_SOURCES} src/realsense/rs_detect.cpp src/realsense/realsense.cpp src/utils/myinfer.cpp src/yolo/yolo.cu)
set(K4A_DETECT_SOURCES ${COMMON_SOURCES} src/k4a/camera_k4a.cpp src/utils/myinfer.cpp src/utils/block_recognizer.cpp src/k4a/k4a_detect.cpp src/yolo/yolo.cu)
set(SERIAL_NODE_SOURCES src/utils/serial.cpp)

# ---- 编译目标 (Non-ROS) ----
add_executable(rs_viewer ${RS_SOURCES})
target_link_libraries(rs_viewer PRIVATE realsense2::realsense2 ${COMMON_LIBS})
target_compile_options(rs_viewer PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)

add_executable(k4a_detect ${K4A_DETECT_SOURCES})
target_link_libraries(k4a_detect PRIVATE k4a::k4a ${COMMON_LIBS})
target_compile_options(k4a_detect PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
# ---- 编译目标 (ROS Support) ----
if(BUILD_WITH_ROS)
    # 1. 设置通用的编译宏
    add_definitions(-DBUILD_WITH_ROS)

    # --- K4A ROS Node ---
    add_executable(k4a_detect_ros ${K4A_DETECT_SOURCES})
    target_compile_options(k4a_detect_ros PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
    
    if(BUILD_WITH_ROS2)
        target_compile_definitions(k4a_detect_ros PRIVATE BUILD_WITH_ROS2)
        # ROS2 必须先调用依赖
        ament_target_dependencies(k4a_detect_ros rclcpp std_msgs sensor_msgs cv_bridge image_transport)
        # 链接本地库和 TensorRT (去掉 PRIVATE)
        target_link_libraries(k4a_detect_ros k4a::k4a ${COMMON_LIBS})
    elseif(BUILD_WITH_ROS1)
        target_compile_definitions(k4a_detect_ros PRIVATE BUILD_WITH_ROS1)
        target_include_directories(k4a_detect_ros PRIVATE ${catkin_INCLUDE_DIRS})
        target_link_libraries(k4a_detect_ros k4a::k4a ${catkin_LIBRARIES} ${COMMON_LIBS})
    endif()

    # --- K4A Capture Images ROS2 Node ---
    add_executable(k4a_capture_images_ros
        src/k4a/k4a_capture_images.cpp
        src/k4a/camera_k4a.cpp
        ${COMMON_SOURCES}
    )

    target_compile_options(k4a_capture_images_ros PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)

    if(BUILD_WITH_ROS2)
        target_compile_definitions(k4a_capture_images_ros PRIVATE BUILD_WITH_ROS2)

        ament_target_dependencies(k4a_capture_images_ros
            rclcpp
            std_msgs
            sensor_msgs
            cv_bridge
            image_transport)

        target_link_libraries(k4a_capture_images_ros
            k4a::k4a
            ${COMMON_LIBS})
    endif()

    # --- RealSense ROS Node ---
    add_executable(rs_viewer_ros ${RS_SOURCES})
    target_compile_options(rs_viewer_ros PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-arch=sm_75>)
    
    if(BUILD_WITH_ROS2)
        target_compile_definitions(rs_viewer_ros PRIVATE BUILD_WITH_ROS2)
        ament_target_dependencies(rs_viewer_ros rclcpp std_msgs sensor_msgs cv_bridge image_transport)
        target_link_libraries(rs_viewer_ros realsense2::realsense2 ${COMMON_LIBS})
    elseif(BUILD_WITH_ROS1)
        target_compile_definitions(rs_viewer_ros PRIVATE BUILD_WITH_ROS1)
        target_include_directories(rs_viewer_ros PRIVATE ${catkin_INCLUDE_DIRS})
        target_link_libraries(rs_viewer_ros realsense2::realsense2 ${catkin_LIBRARIES} ${COMMON_LIBS})
    endif()

    # --- Serial ROS Node ---
    add_executable(k4a_serial_node ${SERIAL_NODE_SOURCES})

    if(BUILD_WITH_ROS2)
        target_compile_definitions(k4a_serial_node PRIVATE BUILD_WITH_ROS2)
        ament_target_dependencies(k4a_serial_node rclcpp std_msgs)
        target_include_directories(k4a_serial_node PRIVATE ${SERIAL_INCLUDE_DIR})
        target_link_libraries(k4a_serial_node ${SERIAL_LIBRARY})
        set_target_properties(k4a_serial_node PROPERTIES
            BUILD_RPATH "${SERIAL_LIBRARY_DIR}"
            INSTALL_RPATH "${SERIAL_LIBRARY_DIR}")
    elseif(BUILD_WITH_ROS1)
        target_compile_definitions(k4a_serial_node PRIVATE BUILD_WITH_ROS1)
        target_include_directories(k4a_serial_node PRIVATE ${catkin_INCLUDE_DIRS})
        target_link_libraries(k4a_serial_node ${catkin_LIBRARIES})
    endif()

    # ---- 核心修正：ROS2 安装逻辑 ----
    if(BUILD_WITH_ROS2)
        # 安装可执行程序到 lib/<package_name> (ROS2 标准)
        install(TARGETS
        k4a_detect_ros
        rs_viewer_ros
        k4a_serial_node
        k4a_capture_images_ros
        DESTINATION lib/${PROJECT_NAME})

        # 安装 Launch 文件夹 (确保目录下有 launch/ 文件夹)
        install(DIRECTORY launch
            DESTINATION share/${PROJECT_NAME})

        # 导出依赖到 ament 索引 (解决找不到包的关键)
        ament_export_dependencies(rclcpp std_msgs sensor_msgs cv_bridge image_transport)
        ament_export_include_directories(inc)
        ament_export_libraries(yolo) # 如果想让别的包调 yolo 库
    endif()

endif()

# ---- 结尾处理 ----
if(BUILD_WITH_ROS2)
    # 必须放在最后
    ament_package()
endif()
