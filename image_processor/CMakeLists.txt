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
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)


#--- for C++
include_directories(include)

function(add_executable_custom exec_name source)
  add_executable(${exec_name} ${source})
  ament_target_dependencies(${exec_name}
    rclcpp
    std_msgs
    sensor_msgs
    std_srvs
    cv_bridge
  )
  install(TARGETS ${exec_name}
    DESTINATION lib/${PROJECT_NAME}
  )
  target_link_libraries(${exec_name} ${OpenCV_LIBS} fmt::fmt)
  target_include_directories(${exec_name} PUBLIC ${OpenCV_INCLUDE_DIRS})
endfunction()

add_executable_custom(compress_image_node main/compress_image_node.cpp)
add_executable_custom(extract_target_node main/extract_target_node.cpp)

#--- end C++

#-- for Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/py_node.py
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

ament_package()
