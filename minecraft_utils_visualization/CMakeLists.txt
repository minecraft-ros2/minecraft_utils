cmake_minimum_required(VERSION 3.5)
project(minecraft_utils_visualization)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(TARGET mob_marker)
ament_auto_add_library(${TARGET} SHARED ./src/${TARGET}.cpp)
rclcpp_components_register_node(
  ${TARGET}
    PLUGIN "minecraft_utils_visualization::MobMarker"
    EXECUTABLE ${TARGET}_exec)
target_include_directories(
  ${TARGET} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()