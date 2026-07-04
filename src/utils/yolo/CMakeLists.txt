cmake_minimum_required(VERSION 3.10)

project(cuda)

if(NOT DEFINED TensorRT_ROOT)
    set(TensorRT_ROOT "/opt/TensorRT-8.6.1.6" CACHE PATH "TensorRT install root")
endif()

set(TENSORRT_LIB_DIR "${TensorRT_ROOT}/lib")
if(EXISTS "${TensorRT_ROOT}/targets/x86_64-linux-gnu/lib")
    set(TENSORRT_LIB_DIR "${TensorRT_ROOT}/targets/x86_64-linux-gnu/lib")
endif()

set(TENSORRT_INCLUDE_DIR "${TensorRT_ROOT}/include")
if(EXISTS "${TensorRT_ROOT}/targets/x86_64-linux-gnu/include")
    set(TENSORRT_INCLUDE_DIR "${TensorRT_ROOT}/targets/x86_64-linux-gnu/include")
endif()

find_library(TENSORRT_NVINFER_LIB nvinfer HINTS ${TENSORRT_LIB_DIR})
find_library(TENSORRT_NVONNXPARSER_LIB nvonnxparser HINTS ${TENSORRT_LIB_DIR})
if(NOT TENSORRT_NVINFER_LIB OR NOT TENSORRT_NVONNXPARSER_LIB)
    message(FATAL_ERROR "TensorRT libs not found in ${TENSORRT_LIB_DIR}. Set TensorRT_ROOT correctly.")
endif()

file(GLOB_RECURSE YOLO_SOURCES "*.cu")
message(STATUS "YOLO_SOURCES: ${YOLO_SOURCES}")

find_package(CUDA REQUIRED)

include_directories(
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
)


cuda_add_library(yolo STATIC ${YOLO_SOURCES})

link_directories(${TENSORRT_LIB_DIR})
target_include_directories(yolo
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../inc
)


target_link_libraries(yolo 
    ${OpenCV_LIBS}
    ${TENSORRT_NVINFER_LIB}
    ${TENSORRT_NVONNXPARSER_LIB}
    cudart
)