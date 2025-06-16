cmake_minimum_required(VERSION 3.8)
project(grasping_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(manipulators REQUIRED)

add_executable(grasp_bag src/grasp_bag.cpp)
target_include_directories(grasp_bag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(grasp_bag PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  grasp_bag
  rclcpp
  manipulators
)

install(TARGETS grasp_bag
  DESTINATION lib/${PROJECT_NAME})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
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
