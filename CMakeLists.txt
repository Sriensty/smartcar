# 包含交叉编译工具链
cmake_minimum_required(VERSION 3.5)
project(project VERSION 0.1.0 LANGUAGES C CXX) # 生成可执行文件名称这里改
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -pthread -Wall")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -Wall")
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE NEVER)
# 使用标准方式查找OpenCV
find_package(OpenCV REQUIRED COMPONENTS
    core
    imgproc
    videoio
    highgui
    imgcodecs
    features2d
)
message(STATUS "OpenCV found in: ${OpenCV_DIR}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "OpenCV includes: ${OpenCV_INCLUDE_DIRS}")

# 包含自定义库和OpenCV头文件
# include_directories(lib /usr/local/include/opencv/install_loongxin/include/opencv4/opencv2)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(lib)
include_directories(/home/smartcar/boost)
include_directories(ZF/zf_common)
include_directories(ZF/zf_components)
include_directories(ZF/zf_device)
include_directories(ZF/zf_driver)

aux_source_directory(src SRC)
aux_source_directory(ZF/zf_common SRC)
aux_source_directory(ZF/zf_components SRC)
aux_source_directory(ZF/zf_device SRC)
aux_source_directory(ZF/zf_driver SRC)

add_executable(${PROJECT_NAME} ${SRC})

# 正确链接所有OpenCV库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})   
