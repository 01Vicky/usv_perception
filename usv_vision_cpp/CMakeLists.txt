cmake_minimum_required(VERSION 3.16)
project(usv_vision_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# usv_interfaces：板上的 CMake config 会强制 find_package(mavros_msgs)（未安装）而直接失败，
# 但我们用到的几个消息本身不依赖 mavros_msgs（typesupport 库里也没有未定义的 mavros 符号），
# 所以不走 find_package，直接连头文件与 typesupport 库。
set(USV_IF_PREFIX "/home/nvidia/usv_ws_test/install/usv_interfaces" CACHE PATH "usv_interfaces 安装前缀")
set(USV_IF_LIBS
  ${USV_IF_PREFIX}/lib/libusv_interfaces__rosidl_typesupport_cpp.so
  ${USV_IF_PREFIX}/lib/libusv_interfaces__rosidl_typesupport_c.so
  ${USV_IF_PREFIX}/lib/libusv_interfaces__rosidl_generator_c.so
  ${USV_IF_PREFIX}/lib/libusv_interfaces__rosidl_typesupport_fastrtps_cpp.so
  ${USV_IF_PREFIX}/lib/libusv_interfaces__rosidl_typesupport_fastrtps_c.so
  ${USV_IF_PREFIX}/lib/libusv_interfaces__rosidl_typesupport_introspection_cpp.so
)

find_package(OpenCV REQUIRED)
find_package(CUDAToolkit REQUIRED)

# TensorRT：Jetson 上装在 /usr/lib/aarch64-linux-gnu/libnvinfer.so
find_library(NVINFER_LIB nvinfer REQUIRED)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin)

add_executable(yolo_infer_cpp_node src/yolo_infer_cpp_node.cpp)
target_include_directories(yolo_infer_cpp_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(yolo_infer_cpp_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${usv_interfaces_LIBRARIES}
  ${USV_IF_LIBS}
  ${OpenCV_LIBS}
  ${NVINFER_LIB}
  CUDA::cudart
)
target_include_directories(yolo_infer_cpp_node PRIVATE ${USV_IF_PREFIX}/include/usv_interfaces)
if(NVINFER_PLUGIN_LIB)
  target_link_libraries(yolo_infer_cpp_node ${NVINFER_PLUGIN_LIB})
endif()

ament_target_dependencies(yolo_infer_cpp_node
  rclcpp sensor_msgs std_msgs cv_bridge)

install(TARGETS yolo_infer_cpp_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
