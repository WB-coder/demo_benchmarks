cmake_minimum_required(VERSION 3.10)
project(iceoryx_demo)

find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_hoofs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(iox_publisher publisher.cpp)
target_link_libraries(iox_publisher iceoryx_posh::iceoryx_posh iceoryx_hoofs::iceoryx_hoofs)

add_executable(iox_subscriber subscriber.cpp)
target_link_libraries(iox_subscriber iceoryx_posh::iceoryx_posh iceoryx_hoofs::iceoryx_hoofs)
