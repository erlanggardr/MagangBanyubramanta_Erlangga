cmake_minimum_required(VERSION 3.8)
project(key_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

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

#  library CheckKeyPressedAction
add_library(check_key_pressed_action src/check_key_pressed_action.cpp)
ament_target_dependencies(check_key_pressed_action rclcpp std_msgs behaviortree_cpp_v3)

# executable untuk key_check_node dan link library check_key_pressed_action
add_executable(key_check_node src/key_check_node.cpp)
ament_target_dependencies(key_check_node rclcpp std_msgs behaviortree_cpp_v3)
target_link_libraries(key_check_node check_key_pressed_action)

# Install targets
install(TARGETS key_check_node check_key_pressed_action
        DESTINATION lib/${PROJECT_NAME})

ament_package()

