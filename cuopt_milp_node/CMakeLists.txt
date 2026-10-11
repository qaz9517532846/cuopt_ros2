cmake_minimum_required(VERSION 3.20)
project(cuopt_milp_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SolveMilp.action"
  DEPENDENCIES action_msgs
)

# ---- cuOpt C API: 從 conda env 找 header/lib，環境變數 CUOPT_SDK_PREFIX 由使用者 export ----
if(NOT DEFINED ENV{CUOPT_SDK_PREFIX})
  message(FATAL_ERROR "請先 export CUOPT_SDK_PREFIX=<conda env 路徑> 再 colcon build")
endif()
set(CUOPT_SDK_PREFIX $ENV{CUOPT_SDK_PREFIX})

find_path(CUOPT_INCLUDE_DIR
  NAMES cuopt/mathematical_optimization/cuopt_c.h cuopt/linear_programming/cuopt_c.h
  HINTS ${CUOPT_SDK_PREFIX}/include
  REQUIRED)
find_library(CUOPT_LIBRARY cuopt
  HINTS ${CUOPT_SDK_PREFIX}/lib REQUIRED)

add_executable(cuopt_milp_action_server src/cuopt_milp_action_server.cpp)
target_include_directories(cuopt_milp_action_server PRIVATE ${CUOPT_INCLUDE_DIR})
target_link_libraries(cuopt_milp_action_server ${CUOPT_LIBRARY})
ament_target_dependencies(cuopt_milp_action_server rclcpp rclcpp_action)
rosidl_target_interfaces(cuopt_milp_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 讓執行檔不靠 LD_LIBRARY_PATH 就能找到 conda env 裡的 libcuopt.so 及其依賴 (libcublas/libcudss/...)
set_target_properties(cuopt_milp_action_server PROPERTIES
  BUILD_RPATH   "${CUOPT_SDK_PREFIX}/lib"
  INSTALL_RPATH "${CUOPT_SDK_PREFIX}/lib"
)

install(TARGETS cuopt_milp_action_server DESTINATION lib/${PROJECT_NAME})
ament_package()
