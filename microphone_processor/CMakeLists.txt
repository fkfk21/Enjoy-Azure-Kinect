cmake_minimum_required(VERSION 3.5)
project(microphone_processor)

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


#--- for C++
include_directories(include)

function(add_executable_custom exec_name source)
  add_executable(${exec_name} ${source})
  ament_target_dependencies(${exec_name}
    rclcpp
    std_msgs
    sensor_msgs
  )
  install(TARGETS ${exec_name}
    DESTINATION lib/${PROJECT_NAME}
  )
  # target_link_libraries()
endfunction()

# add_executable_custom(compress_image_node main/compress_image_node.cpp)
# add_executable_custom(extract_target_node main/extract_target_node.cpp)

#--- end C++

#-- for Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/online_sound_localization.py
  scripts/show_audio_devices.py
  scripts/py_node.py
  scripts/sound.launch.sh
  DESTINATION lib/${PROJECT_NAME}
)
#--- end Python

install(
  DIRECTORY
  # launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

