cmake_minimum_required(VERSION 3.10)
project(soc_fgo)

set(CMAKE_CXX_STANDARD 14)

find_package(GTSAM REQUIRED)

add_executable(soc_fgo main.cpp)
target_link_libraries(soc_fgo gtsam)  # 👈 改为小写 gtsam

