cmake_minimum_required(VERSION 3.10)
project(camera_cxx)

# 添加子目录
add_subdirectory(src/yolo)          # 编译src/yolo目录


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCv_DIR "/home/li/opencv/lib/cmake/opencv4")
set(TensorRT_ROOT "/home/li/TensorRT-8.6.1.6")


# 查找包
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(PCL REQUIRED)
find_package(k4a REQUIRED)
find_package(CUDA REQUIRED)


include_directories(/usr/include
    /usr/local/cuda/include
    ${OpenCv_DIR}/include
    ${TensorRT_ROOT}/include
    ${PCL_INCLUDE_DIRS}/include
)


link_directories(/home/li/camera_cxx/lib)
link_directories(/home/li/TensorRT-8.6.1.6/lib)
link_directories(/usr/lib/x86_64-linux-gnu /usr/local/cuda/lib64)
# 添加可执行文件
add_executable(rs_viewer src/rsmain.cpp src/realsense.cpp src/myinfer.cpp  src/yolo/yolo.cu)

add_executable(k4a_viewer src/k4amain.cpp src/camera_k4a.cpp src/myinfer.cpp src/yolo/yolo.cu)



# 现代CMake方式：使用目标而不是全局变量
target_include_directories(rs_viewer PRIVATE
    inc
)

target_include_directories(k4a_viewer PRIVATE
    inc
)


# 现代链接方式：直接链接目标
target_link_libraries(k4a_viewer PRIVATE
    -lffi # 确保这一行存在
    ${OpenCV_LIBS}
    nvinfer # TensorRT库
    nvonnxparser # ONNX解析器
    ${PCL_LIBRARIES}
    k4a::k4a # 使用现代CMake目标
    ${CUDA_LIBRARIES}
    yolo
)
target_link_libraries(rs_viewer PRIVATE
    -lffi # 确保这一行存在
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    nvinfer # TensorRT库
    nvonnxparser # ONNX解析器
    realsense2::realsense2 # 使用现代CMake目标
    ${CUDA_LIBRARIES}
    yolo
)
