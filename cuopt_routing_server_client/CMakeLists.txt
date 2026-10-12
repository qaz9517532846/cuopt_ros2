cmake_minimum_required(VERSION 3.20)
project(cuopt_routing_server_client)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SolveVrp.action"
  DEPENDENCIES action_msgs
)

# No cuOpt / CUDA / conda dependency at all here -- this package only
# talks HTTP to a standalone cuopt-server (see README for the
# `docker run nvidia/cuopt:latest-cu13` deployment step). That is the
# whole point of this package: it has zero RAPIDS header/ABI coupling.
add_executable(cuopt_routing_action_server src/cuopt_routing_server_client.cpp)
ament_target_dependencies(cuopt_routing_action_server rclcpp rclcpp_action)
target_link_libraries(cuopt_routing_action_server
  CURL::libcurl
  nlohmann_json::nlohmann_json
)
rosidl_target_interfaces(cuopt_routing_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS cuopt_routing_action_server DESTINATION lib/${PROJECT_NAME})
ament_package()
