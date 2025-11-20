cmake_minimum_required(VERSION 3.8)
project(frbot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# 라이브러리 소스 파일들 (필요한 .cpp 다 넣기)
add_library(frbot_hardware SHARED
  frbot_hardware_interface.cpp
)

target_compile_features(frbot_hardware PUBLIC cxx_std_17)

# include 경로: 현재 패키지의 include/ 와 설치 후 include/frbot_hardware
target_include_directories(frbot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(frbot_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  control_msgs
)

# pluginlib 플러그인 xml 내보내기
pluginlib_export_plugin_description_file(
  hardware_interface frbot_hardware_plugin.xml
)

# 라이브러리 설치 (lib/libfrbot_hardware.so)
install(
  TARGETS frbot_hardware
  LIBRARY DESTINATION lib
)

# 헤더 설치 (include/frbot_hardware/...)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 플러그인 xml 설치
install(
  FILES frbot_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
