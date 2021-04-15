cmake_minimum_required(VERSION 3.5)
project(bubble_sort_action_client_plugin)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(algo_msgs REQUIRED)

add_library(bubble_sort_client SHARED
  src/bubble_sort_client.cpp
)
target_include_directories(bubble_sort_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(bubble_sort_client
  PRIVATE "ACTION_EXAMPLE_CPP_BUILDING_DLL"
)
ament_target_dependencies(
  bubble_sort_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "algo_msgs"
)

rclcpp_components_register_node(bubble_sort_client
  PLUGIN "action_example_cpp::BubbleSortActionClient"
  EXECUTABLE bubble_sort_action_client
)

install(TARGETS 
  bubble_sort_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
