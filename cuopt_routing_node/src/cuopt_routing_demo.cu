#include <rclcpp/rclcpp.hpp>
#include <raft/core/handle.hpp>
#include <rmm/device_uvector.hpp>
#include <cuopt/routing/data_model_view.hpp>
#include <cuopt/routing/solver_settings.hpp>
#include <cuopt/routing/solve.hpp>
#include <vector>
#include <cuda_runtime.h>

using namespace cuopt::routing;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cuopt_routing_demo");

  raft::handle_t handle;  // 內含預設 CUDA stream

  const int num_locations = 4;   // depot(0) + 3 個地點
  const int fleet_size = 1;
  const int num_orders = 3;

  std::vector<float> h_cost = {
    0, 10, 15, 20,
    10, 0, 12, 18,
    15, 12, 0, 8,
    20, 18, 8, 0
  };
  std::vector<int> h_orders = {1, 2, 3};

  // host -> device：data_model_view_t 的 setter 吃的是 device pointer
  rmm::device_uvector<float> d_cost(h_cost.size(), handle.get_stream());
  rmm::device_uvector<int> d_orders(h_orders.size(), handle.get_stream());
  cudaMemcpyAsync(d_cost.data(), h_cost.data(), h_cost.size() * sizeof(float),
                   cudaMemcpyHostToDevice, handle.get_stream());
  cudaMemcpyAsync(d_orders.data(), h_orders.data(), h_orders.size() * sizeof(int),
                   cudaMemcpyHostToDevice, handle.get_stream());
  handle.sync_stream();

  data_model_view_t<int, float> data_model(&handle, num_locations, fleet_size, num_orders);
  data_model.add_cost_matrix(d_cost.data());
  data_model.set_order_locations(d_orders.data());

  solver_settings_t<int, float> settings;
  settings.set_time_limit(5.0f);

  assignment_t<int> result = solve(data_model, settings);

  if (result.get_status() == solution_status_t::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "total cost = %f, vehicles used = %d",
                result.get_total_objective(), result.get_vehicle_count());
    // result.get_route() 回傳 rmm::device_uvector<int>&，要 memcpy 回 host 才能印出來
  } else {
    RCLCPP_ERROR(node->get_logger(), "routing solve failed: %s",
                 result.get_status_string().c_str());
  }

  rclcpp::shutdown();
  return 0;
}