cmake_minimum_required(VERSION 3.10)

project(cuda)

set(TENSORRT_DIR "/home/li/TensorRT-8.6.1.6")

file(GLOB_RECURSE YOLO_SOURCES "*.cu")
message(STATUS "YOLO_SOURCES: ${YOLO_SOURCES}")

find_package(CUDA REQUIRED)

include_directories(
    ${TENSORRT_DIR}/include
    ${CUDA_INCLUDE_DIRS}
)


cuda_add_library(yolo STATIC ${YOLO_SOURCES})

link_directories(${TENSORRT_DIR}/lib)

target_link_libraries(yolo 
    ${OpenCV_LIBS}
    nvinfer
    nvonnxparser
    cudart
)