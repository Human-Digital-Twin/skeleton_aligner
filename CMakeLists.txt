cmake_minimum_required(VERSION 3.8)
project(skeleton_aligner)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hiros_skeleton_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${PROJECT_NAME}_node src/main.cpp src/Aligner.cpp src/TfBuffer.cpp src/TfCluster.cpp src/utils.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp hiros_skeleton_msgs tf2 tf2_eigen tf2_geometry_msgs tf2_ros)

set_target_properties(${PROJECT_NAME}_node
  PROPERTIES OUTPUT_NAME
  ${PROJECT_NAME}
  PREFIX
  ""
)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE include
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
