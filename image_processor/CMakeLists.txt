cmake_minimum_required(VERSION 3.5)
project(image_processor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#-- Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)


#--- for C++
include_directories(include)
add_executable(compress_image_node src/compress_image_node.cpp)
ament_target_dependencies(compress_image_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

install(TARGETS
  compress_image_node
  DESTINATION lib/${PROJECT_NAME}
)
#--- end C++

#-- for Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/py_node.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

ament_package()
