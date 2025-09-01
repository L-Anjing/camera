# CMakeLists.txt文件配置的基本逻辑
## 设置阶段-项目基础的配置  
1. `cmake_minimum_required(VERSION 3.10)`CMake最低版本的要求  **（必需）**  
2. `project(xx)`设置项目名称  **（必需）**  
3. `set(CMAKE_CXX_STANDARD 11)`设置C++语言的标准   **（推荐）**  
4. `set(CMAKE_CXX_STANDARD_REQUIRED ON)`设置C++语言的标准是否必须   **（推荐）**  
5. `set（CMAKE_BUILD_TYPE Release）` 设置编译类型为Release   **（推荐）**  
## 找包阶段-第三方库（依赖）的配置
1. `set(OpenCV_DIR "/path/to/opencv/build")`设置依赖库的搜索路径  **（非必需，找不到时可以用）**  
2. `find_package(xxx)`查找依赖库  **（必需）**  
3. ```
    include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${TensorRT_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    include  # 自己的头文件目录)
    ```
   设置依赖库的头文件,以及自己的头文件目录搜索路径  **（必需）**
## 构建阶段-目标构建配置
1. `add_executable(xxx xxx.cpp)`添加目标文件  **（必需）**  
2. `target_link_libraries(xxx ${OpenCV_LIBS} ${TensorRT_LIBS} ${CUDA_LIBS})`链接库文件  **（必需）**  
3. ```
    set_target_properties(my_app PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CXX_STANDARD 14
    )
    ```
   设置编译选项  **（非必需）**# camera 
