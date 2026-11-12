cmake_minimum_required(VERSION 3.16)
project(livox_wall_hazard_detection CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(third_party/Livox-SDK2)

find_package(libwebsockets CONFIG REQUIRED)
# find_package(Opend3D REQUIRED)
find_package(Open3D REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/Open3D) # Use path from dpkg output if different

include_directories(
    include
    include/livox_wall_hazard_detection
    third_party/Livox-SDK2
    third_party/Livox-SDK2/include
    third_party/Livox-SDK2/sdk_core
)

add_executable(livox_server
    src/main.cpp
    src/livox_wall_hazard_detection.cpp
)

target_link_libraries(livox_server PRIVATE
    livox_lidar_sdk_static
    pthread
    websockets
    Open3D::Open3D
)
