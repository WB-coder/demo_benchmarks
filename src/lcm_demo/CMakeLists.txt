cmake_minimum_required(VERSION 3.10)
project(lcm_demo)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# Generate C++ headers from .lcm file
# We assume lcm-gen is in the path.
execute_process(COMMAND lcm-gen -x ${CMAKE_CURRENT_SOURCE_DIR}/timestamp_t.lcm --cpp-std=c++11 OUTPUT_VARIABLE LCM_GEN_OUT)

# Include current directory for generated headers
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${LCM_INCLUDE_DIRS})

add_executable(lcm_publisher publisher.cpp)
target_link_libraries(lcm_publisher ${LCM_LIBRARIES})

add_executable(lcm_subscriber subscriber.cpp)
target_link_libraries(lcm_subscriber ${LCM_LIBRARIES})
