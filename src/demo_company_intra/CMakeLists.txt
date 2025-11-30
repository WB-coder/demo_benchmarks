cmake_minimum_required(VERSION 3.8)
project(demo_company_intra)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)


# Composable node libraries for intra-process communication
add_library(latency_publisher_component SHARED src/latency_publisher_intra.cpp)
ament_target_dependencies(latency_publisher_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(latency_publisher_component
  PLUGIN "demo_company_intra::LatencyPublisher"
  EXECUTABLE latency_publisher_intra)

add_library(latency_listener_component SHARED src/latency_listener_intra.cpp)
ament_target_dependencies(latency_listener_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_node(latency_listener_component
  PLUGIN "demo_company_intra::LatencyListener"
  EXECUTABLE latency_listener_intra)

# Original combined executable (kept for backward compatibility)
add_executable(latency_intra_demo src/latency_intra_demo.cpp)
ament_target_dependencies(latency_intra_demo rclcpp std_msgs)

install(TARGETS
  latency_publisher_component
  latency_listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  latency_intra_demo
  latency_publisher_intra
  latency_listener_intra
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
