cmake_minimum_required(VERSION 3.8)
project(grasping_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# --------------------
# Find dependencies
# --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(manipulators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)


# --------------------
# Libreria con GraspNode
# --------------------
add_library(grasping_utils
  src/grasping_utils.cpp
)
target_include_directories(grasping_utils PUBLIC include)
ament_target_dependencies(grasping_utils
  rclcpp
  ament_index_cpp
  geometry_msgs
  std_msgs
  visualization_msgs
  moveit_ros_planning_interface
  manipulators
)

# --------------------
# Executables
# --------------------

# grasp_bag
add_executable(grasp_bag src/grasp_bag.cpp)
target_compile_features(grasp_bag PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(grasp_bag
  rclcpp
  std_msgs
  geometry_msgs
  manipulators
  moveit_msgs
  moveit_ros_planning_interface
)
target_link_libraries(grasp_bag grasping_utils)

# auto_grasp_bag
add_executable(auto_grasp_bag src/auto_grasp_bag.cpp)
target_compile_features(auto_grasp_bag PUBLIC c_std_99 cxx_std_17) 
ament_target_dependencies(auto_grasp_bag
  rclcpp
  std_msgs
  geometry_msgs
  manipulators
  tf2_ros
  moveit_msgs
  moveit_ros_planning_interface
)
target_link_libraries(auto_grasp_bag grasping_utils)

# --------------------
# Include directories (per tutti)
# --------------------
target_include_directories(grasp_bag PUBLIC include)
target_include_directories(auto_grasp_bag PUBLIC include)
target_include_directories(grasping_utils PUBLIC include)

# --------------------
# Install targets
# --------------------
install(TARGETS
  grasp_bag
  auto_grasp_bag
  grasping_utils
  DESTINATION lib/${PROJECT_NAME}
)
  
# --------------------
# Install launch & config
# --------------------
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)



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
