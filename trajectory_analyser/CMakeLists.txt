cmake_minimum_required(VERSION 3.8)
project(trajectory_analyser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(custom_service REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(trajectory_saver_node src/trajectory_saver.cpp)
add_executable(trajectory_viewer_node src/trajectory_viewer.cpp)
ament_target_dependencies(trajectory_saver_node rclcpp std_msgs nav_msgs custom_service)
ament_target_dependencies(trajectory_viewer_node rclcpp std_msgs nav_msgs custom_service visualization_msgs)

target_link_libraries(trajectory_saver_node yaml-cpp)
target_link_libraries(trajectory_viewer_node yaml-cpp)

install(TARGETS
  trajectory_saver_node
  trajectory_viewer_node
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
