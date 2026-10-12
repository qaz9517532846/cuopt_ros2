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

// Mirrors the official pdptw_mixed_fleet_basic_mixed_fleet_scenario() in
// cpp/tests/examples/routing/pdptw_mixed_fleet.cu: a mixed fleet (small/large
// trucks) serving pickup-delivery pairs with time windows and capacity.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pdptw_mixed_fleet_demo");

  raft::handle_t handle;
  auto stream = handle.get_stream();

  const int num_locations = 5;  // depot (0) + 4 pickup/delivery points
  const int fleet_size = 3;     // 2 small trucks + 1 large truck
  const int num_orders = 4;     // 2 pickup-delivery pairs

  std::vector<float> h_cost_matrix = {
    0,  10, 15, 20, 25,
    10, 0,  35, 25, 30,
    15, 35, 0,  30, 20,
    20, 25, 30, 0,  15,
    25, 30, 20, 15, 0
  };

  // Vehicle types: 0 = small truck, 1 = large truck.
  std::vector<uint8_t> h_vehicle_types = {0, 0, 1};

  std::vector<int> h_order_locations = {1, 2, 3, 4};

  // Pickup/delivery pairs, indexed into the order arrays (not location ids):
  // order 0 is picked up, order 1 is its matching delivery; same for 2/3.
  std::vector<int> h_pickup_orders = {0, 2};
  std::vector<int> h_delivery_orders = {1, 3};

  std::vector<int> h_order_earliest = {0, 0, 0, 0};
  std::vector<int> h_order_latest = {100, 100, 100, 100};
  std::vector<int> h_service_times = {10, 10, 10, 10};

  // Capacities: small trucks 100, large truck 200. Demands: positive at
  // pickup, negative (matching magnitude) at the paired delivery.
  std::vector<int> h_capacities = {100, 100, 200};
  std::vector<int> h_demands = {50, -50, 150, -150};

  std::vector<int> h_vehicle_start_locations = {0, 0, 0};
  std::vector<int> h_vehicle_end_locations = {0, 0, 0};

  auto v_cost_matrix = cuopt::device_copy(h_cost_matrix, stream);
  auto v_vehicle_types = cuopt::device_copy(h_vehicle_types, stream);
  auto v_order_locations = cuopt::device_copy(h_order_locations, stream);
  auto v_pickup_orders = cuopt::device_copy(h_pickup_orders, stream);
  auto v_delivery_orders = cuopt::device_copy(h_delivery_orders, stream);
  auto v_order_earliest = cuopt::device_copy(h_order_earliest, stream);
  auto v_order_latest = cuopt::device_copy(h_order_latest, stream);
  auto v_service_times = cuopt::device_copy(h_service_times, stream);
  auto v_capacities = cuopt::device_copy(h_capacities, stream);
  auto v_demands = cuopt::device_copy(h_demands, stream);
  auto v_start_locations = cuopt::device_copy(h_vehicle_start_locations, stream);
  auto v_end_locations = cuopt::device_copy(h_vehicle_end_locations, stream);

  data_model_view_t<int, float> data_model(&handle, num_locations, fleet_size, num_orders);
  data_model.set_vehicle_types(v_vehicle_types.data());
  // add_cost_matrix() is keyed by vehicle_type (default 0). Once set_vehicle_types()
  // declares more than one type, every type needs its own call -- the solver rejects
  // the model with "All vehicle cost matrices should be set" otherwise. Both types use
  // the same road network/distances here, so the same device buffer is passed twice.
  data_model.add_cost_matrix(v_cost_matrix.data(), 0);
  data_model.add_cost_matrix(v_cost_matrix.data(), 1);
  data_model.set_order_locations(v_order_locations.data());
  data_model.set_pickup_delivery_pairs(v_pickup_orders.data(), v_delivery_orders.data());
  data_model.set_order_time_windows(v_order_earliest.data(), v_order_latest.data());
  data_model.set_order_service_times(v_service_times.data());
  data_model.set_vehicle_locations(v_start_locations.data(), v_end_locations.data());
  data_model.add_capacity_dimension("demand", v_demands.data(), v_capacities.data());

  solver_settings_t<int, float> settings;
  settings.set_time_limit(10.0f);

  assignment_t<int> result = solve(data_model, settings);
  handle.sync_stream();

  if (result.get_status() == solution_status_t::SUCCESS) {
    host_assignment_t<int> host_route(result);

    RCLCPP_INFO(node->get_logger(), "total cost = %f, vehicles used = %d",
                result.get_total_objective(), result.get_vehicle_count());

    int current_truck = -1;
    std::string route_str;
    for (size_t i = 0; i < host_route.locations.size(); ++i) {
      if (host_route.truck_id[i] != current_truck) {
        if (current_truck != -1) {
          const int vt = h_vehicle_types[current_truck];
          RCLCPP_INFO(node->get_logger(), "vehicle %d (type=%d, %s truck) route: %s",
                      current_truck, vt, vt == 1 ? "large" : "small", route_str.c_str());
        }
        current_truck = host_route.truck_id[i];
        route_str.clear();
      }
      route_str += std::to_string(host_route.locations[i]) + " ";
    }
    if (current_truck != -1) {
      const int vt = h_vehicle_types[current_truck];
      RCLCPP_INFO(node->get_logger(), "vehicle %d (type=%d, %s truck) route: %s",
                  current_truck, vt, vt == 1 ? "large" : "small", route_str.c_str());
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "routing solve failed: %s (detail: %s)",
                 result.get_status_string().c_str(), result.get_error_status().what());
  }

  rclcpp::shutdown();
  return 0;
}
