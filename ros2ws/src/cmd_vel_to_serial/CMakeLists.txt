cmake_minimum_required(VERSION 3.8)
project(cmd_vel_to_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

# Add executable
add_executable(cmd_vel_to_serial src/main.cpp)

# Specify dependencies of the executable
ament_target_dependencies(cmd_vel_to_serial
  rclcpp
  interfaces
)


# Install the executable
install(TARGETS
  cmd_vel_to_serial
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

