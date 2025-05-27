cmake_minimum_required(VERSION 3.14)
project(autoware_tensorrt_vad)

find_package(autoware_cmake REQUIRED)
autoware_package()

add_compile_options(-Wno-deprecated-declarations)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Find additional dependencies for components
find_package(rclcpp_components REQUIRED)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  lib/vad_trt.cpp
  # TODO(Shin-kyoto): Add your library source files here when implemented
  # lib/vad_interface.cpp
  # lib/vad_config.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/vad_node.cpp
)

target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_lib
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::tensorrt_vad::VadNode"
  EXECUTABLE vad_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
