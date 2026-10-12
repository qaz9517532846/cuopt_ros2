#include <rclcpp/rclcpp.hpp>
#include <raft/core/handle.hpp>
#include <rmm/device_uvector.hpp>
#include <cuopt/routing/data_model_view.hpp>
#include <cuopt/routing/solver_settings.hpp>
#include <cuopt/routing/solve.hpp>
#include <cuopt_routing_node/copy_helpers.hpp>
#include <string>
#include <vector>

using namespace cuopt::routing;

// Mirrors the official basic_service_team_scenario() in
// cpp/tests/examples/routing/service_team_routing.cu: service technicians with
// time windows plus a mandatory break per technician, taken at the depot.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("service_team_routing_demo");

  raft::handle_t handle;
  auto stream = handle.get_stream();

  const int num_locations = 5;  // depot (0) + 4 service points
  const int fleet_size = 3;     // 3 service teams
  const int num_orders = 4;     // 4 service requests

  std::vector<float> h_cost_matrix = {
    0,  10, 15, 20, 25,
    10, 0,  35, 25, 30,
    15, 35, 0,  30, 20,
    20, 25, 30, 0,  15,
    25, 30, 20, 15, 0
  };
  std::vector<int> h_order_locations = {1, 2, 3, 4};
  std::vector<int> h_order_earliest = {0, 0, 0, 0};
  std::vector<int> h_order_latest = {100, 100, 100, 100};
  std::vector<int> h_service_times = {30, 45, 60, 30};

  // One break per vehicle: earliest/latest/duration arrays are sized by
  // fleet_size. break_locations is different -- it is the *set* of eligible
  // break locations shared across all vehicles (installed libcuopt rejects
  // duplicates: "There should be unique break locations"), not one entry per
  // vehicle as the upstream example's comment implies. All three teams can
  // only break at the depot, so that set has exactly one element.
  std::vector<int> h_break_earliest = {30, 30, 30};
  std::vector<int> h_break_latest = {70, 70, 70};
  std::vector<int> h_break_duration = {15, 15, 15};
  std::vector<int> h_break_locations = {0};

  std::vector<int> h_vehicle_start_locations = {0, 0, 0};
  std::vector<int> h_vehicle_end_locations = {0, 0, 0};

  auto v_cost_matrix = cuopt::device_copy(h_cost_matrix, stream);
  auto v_order_locations = cuopt::device_copy(h_order_locations, stream);
  auto v_order_earliest = cuopt::device_copy(h_order_earliest, stream);
  auto v_order_latest = cuopt::device_copy(h_order_latest, stream);
  auto v_service_times = cuopt::device_copy(h_service_times, stream);
  auto v_break_earliest = cuopt::device_copy(h_break_earliest, stream);
  auto v_break_latest = cuopt::device_copy(h_break_latest, stream);
  auto v_break_duration = cuopt::device_copy(h_break_duration, stream);
  auto v_break_locations = cuopt::device_copy(h_break_locations, stream);
  auto v_start_locations = cuopt::device_copy(h_vehicle_start_locations, stream);
  auto v_end_locations = cuopt::device_copy(h_vehicle_end_locations, stream);

  data_model_view_t<int, float> data_model(&handle, num_locations, fleet_size, num_orders);
  data_model.add_cost_matrix(v_cost_matrix.data());
  data_model.set_order_locations(v_order_locations.data());
  data_model.set_order_time_windows(v_order_earliest.data(), v_order_latest.data());
  data_model.set_order_service_times(v_service_times.data());
  data_model.set_vehicle_locations(v_start_locations.data(), v_end_locations.data());
  data_model.add_break_dimension(v_break_earliest.data(), v_break_latest.data(),
                                 v_break_duration.data());
  data_model.set_break_locations(v_break_locations.data(),
                                 static_cast<int>(v_break_locations.size()));

  solver_settings_t<int, float> settings;
  settings.set_time_limit(10.0f);

  assignment_t<int> result = solve(data_model, settings);
  handle.sync_stream();

  if (result.get_status() == solution_status_t::SUCCESS) {
    host_assignment_t<int> host_route(result);

    RCLCPP_INFO(node->get_logger(), "total cost = %f, teams used = %d",
                result.get_total_objective(), result.get_vehicle_count());

    // host_route.locations holds the physical location at each stop (depot
    // included); host_route.node_types tags what kind of stop it is (depot /
    // service call / break) -- printed as a raw int since this installed
    // header does not expose the enum's human names.
    int current_truck = -1;
    std::string route_str;
    for (size_t i = 0; i < host_route.locations.size(); ++i) {
      if (host_route.truck_id[i] != current_truck) {
        if (current_truck != -1) {
          RCLCPP_INFO(node->get_logger(), "team %d route: %s", current_truck, route_str.c_str());
        }
        current_truck = host_route.truck_id[i];
        route_str.clear();
      }
      route_str += std::to_string(host_route.locations[i]) + "(type=" +
                   std::to_string(host_route.node_types[i]) + ") ";
    }
    if (current_truck != -1) {
      RCLCPP_INFO(node->get_logger(), "team %d route: %s", current_truck, route_str.c_str());
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "routing solve failed: %s",
                 result.get_status_string().c_str());
  }

  rclcpp::shutdown();
  return 0;
}
