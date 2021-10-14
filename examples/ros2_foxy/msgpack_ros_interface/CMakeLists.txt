cmake_minimum_required(VERSION 3.5)
project(msgpack_ros_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
# find msgs packages
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# set include directory
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  # msgs
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

# add executable
add_executable(${PROJECT_NAME} src/msgpack_ros_interface.cpp)

# target dependencies
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "serial"
  # msgs
  "geometry_msgs"
  "nav_msgs"
)

# install executable to lib
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# install include directory
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h(pp)?"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
