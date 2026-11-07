cmake_minimum_required(VERSION 3.8)
project(usv_vision_gmsl_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找ROS 2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# Jetson 上 nvcc 不在 PATH，显式指定编译器与目标架构（Orin = sm_87）
if(NOT DEFINED CMAKE_CUDA_COMPILER)
  if(EXISTS "/usr/local/cuda-12.6/bin/nvcc")
    set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.6/bin/nvcc")
  elseif(EXISTS "/usr/local/cuda/bin/nvcc")
    set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
  endif()
endif()
set(CMAKE_CUDA_ARCHITECTURES 87)
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-1.0)

# Miivii GMSL SDK路径
set(MIIVII_INCLUDE_DIR "/opt/miivii/include")
set(MIIVII_LIB_DIR "/opt/miivii/lib")

# 检查GMSL SDK是否存在
if(EXISTS ${MIIVII_INCLUDE_DIR}/MvGmslCamera.h AND EXISTS ${MIIVII_LIB_DIR}/libmvgmslcamera.so)
  set(BUILD_GMSL_CPP_NODE TRUE)
  message(STATUS "Miivii GMSL SDK found at ${MIIVII_INCLUDE_DIR}, building C++ node")
else()
  set(BUILD_GMSL_CPP_NODE FALSE)
  message(WARNING "Miivii GMSL SDK not found. Skipping C++ node build.")
  message(WARNING "  Expected: ${MIIVII_INCLUDE_DIR}/MvGmslCamera.h")
  message(WARNING "  Expected: ${MIIVII_LIB_DIR}/libmvgmslcamera.so")
endif()

# 条件编译C++节点
if(BUILD_GMSL_CPP_NODE)
  # nlohmann_json for JSON handling
  find_package(nlohmann_json QUIET)
  if(NOT nlohmann_json_FOUND)
    message(WARNING "nlohmann_json not found. Install with: sudo apt install nlohmann-json3-dev")
    message(WARNING "Attempting to build without nlohmann_json...")
  endif()

  # 单相机节点
  add_library(algo_resize_cuda STATIC src/algo_resize_cuda.cu)
  target_include_directories(algo_resize_cuda PUBLIC ${CUDAToolkit_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
  target_link_libraries(algo_resize_cuda PUBLIC CUDA::cudart)

  add_library(h264_streamer STATIC src/h264_streamer.cpp)
  target_include_directories(h264_streamer PUBLIC ${GSTREAMER_INCLUDE_DIRS} include)
  target_link_libraries(h264_streamer PUBLIC ${GSTREAMER_LIBRARIES} ${OpenCV_LIBS})

  add_executable(gmsl_camera_node_cpp src/gmsl_camera_node_cpp.cpp)

  target_include_directories(gmsl_camera_node_cpp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MIIVII_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
  )

  ament_target_dependencies(gmsl_camera_node_cpp
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
  )

  target_link_libraries(gmsl_camera_node_cpp h264_streamer algo_resize_cuda
    ${OpenCV_LIBRARIES}
    ${MIIVII_LIB_DIR}/libmvgmslcamera.so
  )

  if(nlohmann_json_FOUND)
    target_link_libraries(gmsl_camera_node_cpp nlohmann_json::nlohmann_json)
  endif()

  # 多相机硬件同步节点
  add_executable(gmsl_multi_camera_sync_node src/gmsl_multi_camera_sync_node.cpp)

  target_include_directories(gmsl_multi_camera_sync_node PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MIIVII_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
  )

  ament_target_dependencies(gmsl_multi_camera_sync_node
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
  )

  target_link_libraries(gmsl_multi_camera_sync_node h264_streamer algo_resize_cuda
    ${OpenCV_LIBRARIES}
    ${MIIVII_LIB_DIR}/libmvgmslcamera.so
  )

  if(nlohmann_json_FOUND)
    target_link_libraries(gmsl_multi_camera_sync_node nlohmann_json::nlohmann_json)
  endif()

  install(TARGETS
    gmsl_camera_node_cpp
    gmsl_multi_camera_sync_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# 安装配置文件和launch文件
install(DIRECTORY
  config
  launch
  docs
  DESTINATION share/${PROJECT_NAME}
)

# 安装头文件（供包内节点共用）
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
