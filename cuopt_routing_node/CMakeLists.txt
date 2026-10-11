cmake_minimum_required(VERSION 3.20)
project(cuopt_routing_node LANGUAGES CXX CUDA)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

if(NOT DEFINED ENV{CUOPT_SDK_PREFIX})
  message(FATAL_ERROR "請先 export CUOPT_SDK_PREFIX=<conda env 路徑>")
endif()
set(CUOPT_SDK_PREFIX $ENV{CUOPT_SDK_PREFIX})

# 讓 find_package(rmm/raft CONFIG) 找到 conda env 裡的 *-config.cmake
list(APPEND CMAKE_PREFIX_PATH ${CUOPT_SDK_PREFIX})

set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_ARCHITECTURES native)

# 用 find_package(CONFIG) 而不是手動塞 include/lib path --
# 這兩行會自動帶對 RMM 要的 CCCL 3.2.0 路徑，以及
# -DLIBCUDACXX_ENABLE_EXPERIMENTAL_MEMORY_RESOURCE 這個編譯定義
find_package(rmm CONFIG REQUIRED)
find_package(raft CONFIG REQUIRED)

find_path(CUOPT_INCLUDE_DIR
  NAMES cuopt/routing/solve.hpp
  HINTS ${CUOPT_SDK_PREFIX}/include
  REQUIRED)
find_library(CUOPT_LIBRARY cuopt
  HINTS ${CUOPT_SDK_PREFIX}/lib REQUIRED)

add_executable(cuopt_routing_demo src/cuopt_routing_demo.cu)
target_include_directories(cuopt_routing_demo PRIVATE ${CUOPT_INCLUDE_DIR})
target_link_libraries(cuopt_routing_demo
  ${CUOPT_LIBRARY}
  rmm::rmm
  raft::raft
)
ament_target_dependencies(cuopt_routing_demo rclcpp)

set_target_properties(cuopt_routing_demo PROPERTIES
  BUILD_RPATH   "${CUOPT_SDK_PREFIX}/lib"
  INSTALL_RPATH "${CUOPT_SDK_PREFIX}/lib"
)

install(TARGETS cuopt_routing_demo DESTINATION lib/${PROJECT_NAME})
ament_package()