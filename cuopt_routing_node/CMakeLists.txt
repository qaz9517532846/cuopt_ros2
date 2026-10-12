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

# Three demos, one per official cuOpt C++ routing example
# (cpp/tests/examples/routing/): CVRPTW, mixed-fleet PDPTW, and service-team
# routing with breaks. All share the same include/link setup.
set(ROUTING_DEMO_TARGETS
  cuopt_routing_demo
  pdptw_mixed_fleet_demo
  service_team_routing_demo
)

foreach(target ${ROUTING_DEMO_TARGETS})
  add_executable(${target} src/${target}.cu)
  target_include_directories(${target} PRIVATE
    ${CUOPT_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include   # vendored include/cuopt_routing_node/copy_helpers.hpp
  )
  target_link_libraries(${target}
    ${CUOPT_LIBRARY}
    rmm::rmm
    raft::raft
  )
  ament_target_dependencies(${target} rclcpp)

  set_target_properties(${target} PROPERTIES
    BUILD_RPATH   "${CUOPT_SDK_PREFIX}/lib"
    INSTALL_RPATH "${CUOPT_SDK_PREFIX}/lib"
  )

  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_package()