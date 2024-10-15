cmake_minimum_required(VERSION 3.8)
project(my_robot_package)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include and link directories for MySQL
include_directories(/usr/include/mysql)
link_directories(/usr/lib/mysql)

# Declare a C++ executable
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node
  rclcpp
  std_msgs
)
target_link_libraries(my_node mysqlclient)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME}
)

# Package configuration
ament_package()

