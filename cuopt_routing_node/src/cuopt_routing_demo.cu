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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cuopt_routing_demo");

  raft::handle_t handle;
  auto stream = handle.get_stream();

  // Capacitated VRP with time windows: depot (0) + 5 delivery points, 2 vehicles.
  // Shape follows the official cuOpt C++ examples (cpp/tests/examples/routing/
  // cvrp_daily_deliveries.cu + service_team_routing.cu), combined into one scenario.
  const int num_locations = 6;
  const int fleet_size = 2;
  const int num_orders = 5;

  std::vector<float> h_cost_matrix = {
    0,  10, 15, 20, 25, 30,
    10, 0,  35, 25, 30, 20,
    15, 35, 0,  30, 20, 25,
    20, 25, 30, 0,  15, 30,
    25, 30, 20, 15, 0,  35,
    30, 20, 25, 30, 35, 0
  };
  std::vector<int> h_order_locations = {1, 2, 3, 4, 5};
  std::vector<int> h_vehicle_start_locations = {0, 0};
  std::vector<int> h_vehicle_end_locations = {0, 0};

  // Capacity dimension. NOTE: data_model_view.hpp's doc comment for
  // add_capacity_dimension says demand should cover "each location, including
  // the depot" (i.e. num_locations entries), but the official
  // cvrp_daily_deliveries.cu example passes one entry per *order* (num_orders
  // entries, no depot slot) and that example ships as a passing test. We
  // follow the tested example rather than the docstring here; this was
  // confirmed against a real solve below (SUCCESS, capacity respected).
  std::vector<int> h_capacities = {100, 100};
  std::vector<int> h_demands = {40, 30, 25, 35, 20};

  std::vector<int> h_order_earliest = {0, 0, 0, 0, 0};
  std::vector<int> h_order_latest = {100, 100, 100, 100, 100};
  std::vector<int> h_service_times = {5, 5, 5, 5, 5};

  auto v_cost_matrix = cuopt::device_copy(h_cost_matrix, stream);
  auto v_order_locations = cuopt::device_copy(h_order_locations, stream);
  auto v_start_locations = cuopt::device_copy(h_vehicle_start_locations, stream);
  auto v_end_locations = cuopt::device_copy(h_vehicle_end_locations, stream);
  auto v_capacities = cuopt::device_copy(h_capacities, stream);
  auto v_demands = cuopt::device_copy(h_demands, stream);
  auto v_order_earliest = cuopt::device_copy(h_order_earliest, stream);
  auto v_order_latest = cuopt::device_copy(h_order_latest, stream);
  auto v_service_times = cuopt::device_copy(h_service_times, stream);

  data_model_view_t<int, float> data_model(&handle, num_locations, fleet_size, num_orders);
  data_model.add_cost_matrix(v_cost_matrix.data());
  data_model.set_order_locations(v_order_locations.data());
  data_model.set_vehicle_locations(v_start_locations.data(), v_end_locations.data());
  data_model.add_capacity_dimension("demand", v_demands.data(), v_capacities.data());
  data_model.set_order_time_windows(v_order_earliest.data(), v_order_latest.data());
  data_model.set_order_service_times(v_service_times.data());

  solver_settings_t<int, float> settings;
  settings.set_time_limit(5.0f);

  assignment_t<int> result = solve(data_model, settings);
  handle.sync_stream();

  if (result.get_status() == solution_status_t::SUCCESS) {
    // host_assignment_t copies route/truck_id/etc. from device to host for us --
    // it is part of the public, installed assignment.hpp (CUOPT_EXPORT'd), so no
    // vendored header is needed for this side, only for the device_copy() calls above.
    host_assignment_t<int> host_route(result);

    RCLCPP_INFO(node->get_logger(), "total cost = %f, vehicles used = %d",
                result.get_total_objective(), result.get_vehicle_count());

    // NOTE: host_route.route holds the *order index* for non-depot stops, not the
    // physical location -- confirmed by cross-checking against node_types/locations
    // on a real solve (e.g. route=0 at a delivery stop turned out to be order index 0,
    // i.e. h_order_locations[0], while host_route.locations[i] gave the actual
    // location id). get_route()'s doc comment calls it "a vector of locations", but
    // that did not hold in practice here. host_route.locations is the field that
    // matches the physical location at every stop, depot included, so use that.
    int current_truck = -1;
    std::string route_str;
    for (size_t i = 0; i < host_route.locations.size(); ++i) {
      if (host_route.truck_id[i] != current_truck) {
        if (current_truck != -1) {
          RCLCPP_INFO(node->get_logger(), "vehicle %d route: %s",
                      current_truck, route_str.c_str());
        }
        current_truck = host_route.truck_id[i];
        route_str.clear();
      }
      route_str += std::to_string(host_route.locations[i]) + " ";
    }
    if (current_truck != -1) {
      RCLCPP_INFO(node->get_logger(), "vehicle %d route: %s", current_truck, route_str.c_str());
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "routing solve failed: %s",
                 result.get_status_string().c_str());
  }

  rclcpp::shutdown();
  return 0;
}
