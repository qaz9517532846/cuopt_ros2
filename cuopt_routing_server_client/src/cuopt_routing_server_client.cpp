#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "cuopt_routing_server_client/action/solve_vrp.hpp"

using SolveVrp = cuopt_routing_server_client::action::SolveVrp;
using GoalHandle = rclcpp_action::ServerGoalHandle<SolveVrp>;
using json = nlohmann::json;

namespace {

struct HttpResponse {
  long status_code = 0;
  std::string body;
};

size_t curl_write_cb(void* contents, size_t size, size_t nmemb, void* userp) {
  auto* out = static_cast<std::string*>(userp);
  out->append(static_cast<char*>(contents), size * nmemb);
  return size * nmemb;
}

HttpResponse http_post_json(const std::string& url, const std::string& body) {
  CURL* curl = curl_easy_init();
  if (!curl) throw std::runtime_error("curl_easy_init failed");

  std::string response;
  struct curl_slist* headers = curl_slist_append(nullptr, "Content-Type: application/json");
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

  CURLcode res = curl_easy_perform(curl);
  long status_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    throw std::runtime_error(std::string("POST failed: ") + curl_easy_strerror(res));
  }
  return {status_code, response};
}

HttpResponse http_get(const std::string& url) {
  CURL* curl = curl_easy_init();
  if (!curl) throw std::runtime_error("curl_easy_init failed");

  std::string response;
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

  CURLcode res = curl_easy_perform(curl);
  long status_code = 0;
  curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    throw std::runtime_error(std::string("GET failed: ") + curl_easy_strerror(res));
  }
  return {status_code, response};
}

json to_matrix(const std::vector<double>& flat, int n) {
  json m = json::array();
  for (int i = 0; i < n; ++i) {
    json row = json::array();
    for (int j = 0; j < n; ++j) row.push_back(flat[static_cast<size_t>(i) * n + j]);
    m.push_back(row);
  }
  return m;
}

json to_pairs(const std::vector<double>& earliest, const std::vector<double>& latest) {
  json arr = json::array();
  for (size_t i = 0; i < earliest.size(); ++i) arr.push_back({earliest[i], latest[i]});
  return arr;
}

json to_int_pairs(const std::vector<int32_t>& a, const std::vector<int32_t>& b) {
  json arr = json::array();
  for (size_t i = 0; i < a.size(); ++i) arr.push_back({a[i], b[i]});
  return arr;
}

// Server returns reqId as an opaque token that we interpolate into the next
// GET URL. Validate it first -- same rule the cuOpt server skill documents
// for its Python reference client.
bool is_valid_req_id(const std::string& req_id) {
  static const std::regex pattern("^[A-Za-z0-9_-]{1,64}$");
  return std::regex_match(req_id, pattern);
}

}  // namespace

class CuoptRoutingServerClient : public rclcpp::Node {
 public:
  CuoptRoutingServerClient() : Node("cuopt_routing_server_client") {
    server_url_ = this->declare_parameter<std::string>("server_url", "http://localhost:8000");

    server_ = rclcpp_action::create_server<SolveVrp>(
      this, "solve_vrp",
      std::bind(&CuoptRoutingServerClient::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CuoptRoutingServerClient::handle_cancel, this, std::placeholders::_1),
      std::bind(&CuoptRoutingServerClient::handle_accepted, this, std::placeholders::_1));

    check_server_health();
  }

 private:
  rclcpp_action::Server<SolveVrp>::SharedPtr server_;
  std::string server_url_;

  void check_server_health() {
    try {
      auto resp = http_get(server_url_ + "/cuopt/health");
      if (resp.status_code == 200) {
        RCLCPP_INFO(get_logger(), "cuopt-server reachable at %s", server_url_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "cuopt-server health check at %s returned HTTP %ld",
                    server_url_.c_str(), resp.status_code);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(),
                  "cuopt-server not reachable at %s (%s) -- node will still accept goals "
                  "and retry the HTTP call when one arrives",
                  server_url_.c_str(), e.what());
    }
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                           std::shared_ptr<const SolveVrp::Goal> goal) {
    if (goal->num_locations <= 0 ||
        goal->cost_matrix.size() != static_cast<size_t>(goal->num_locations) * goal->num_locations) {
      RCLCPP_WARN(get_logger(), "rejecting goal: cost_matrix size must equal num_locations^2");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> gh) {
    std::thread{[this, gh] { execute(gh); }}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<SolveVrp::Result>();
    auto feedback = std::make_shared<SolveVrp::Feedback>();

    const int n = goal->num_locations;
    const std::string server = goal->server_url.empty() ? server_url_ : goal->server_url;

    json payload;
    payload["cost_matrix_data"]["data"]["0"] = to_matrix(goal->cost_matrix, n);
    const auto& ttm = goal->travel_time_matrix.empty() ? goal->cost_matrix : goal->travel_time_matrix;
    payload["travel_time_matrix_data"]["data"]["0"] = to_matrix(ttm, n);

    payload["task_data"]["task_locations"] = goal->task_locations;
    payload["task_data"]["demand"] = json::array({goal->demand});
    payload["task_data"]["task_time_windows"] =
      to_pairs(goal->task_time_windows_earliest, goal->task_time_windows_latest);
    payload["task_data"]["service_times"] = goal->service_times;

    payload["fleet_data"]["vehicle_locations"] =
      to_int_pairs(goal->vehicle_start_locations, goal->vehicle_end_locations);
    payload["fleet_data"]["capacities"] = json::array({goal->vehicle_capacities});
    payload["fleet_data"]["vehicle_time_windows"] =
      to_pairs(goal->vehicle_time_windows_earliest, goal->vehicle_time_windows_latest);

    payload["solver_config"]["time_limit"] = goal->solver_time_limit;

    std::string req_id;
    try {
      feedback->status = "submitting to " + server;
      goal_handle->publish_feedback(feedback);

      auto resp = http_post_json(server + "/cuopt/request", payload.dump());
      if (resp.status_code != 200) {
        result->success = false;
        result->error_message = "server rejected request: HTTP " + std::to_string(resp.status_code) +
                                 " body=" + resp.body;
        goal_handle->abort(result);
        return;
      }
      auto resp_json = json::parse(resp.body);
      req_id = resp_json.at("reqId").get<std::string>();
    } catch (const std::exception& e) {
      result->success = false;
      result->error_message = std::string("submit failed: ") + e.what();
      goal_handle->abort(result);
      return;
    }

    if (!is_valid_req_id(req_id)) {
      result->success = false;
      result->error_message = "server returned an unexpected reqId";
      goal_handle->abort(result);
      return;
    }

    const int max_attempts = std::max(30, static_cast<int>(goal->solver_time_limit) + 30);
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->error_message = "canceled by client";
        goal_handle->canceled(result);
        return;
      }

      feedback->status = "polling reqId=" + req_id + " (" + std::to_string(attempt + 1) + "/" +
                          std::to_string(max_attempts) + ")";
      goal_handle->publish_feedback(feedback);

      json resp_json;
      try {
        auto resp = http_get(server + "/cuopt/solution/" + req_id);
        resp_json = json::parse(resp.body, nullptr, /*allow_exceptions=*/false);
      } catch (const std::exception&) {
        // transient network error while polling -- retry rather than fail the whole goal
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      if (resp_json.is_discarded()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      if (resp_json.contains("response")) {
        // Wrapped defensively: the server's response shape is not guaranteed (e.g. "status"
        // comes back as a *number* -- 0=SUCCESS,1=FAIL,2=TIMEOUT,3=EMPTY, matching the Python
        // routing API's solution_status_t, not the string this code originally assumed). Any
        // field-access exception here must not escape: this runs on a detached thread, and an
        // uncaught exception would call std::terminate() and kill the whole node process,
        // taking down every other in-flight goal with it.
        try {
          json solver_response = resp_json["response"].value("solver_response", json::object());

          int status_code = -1;
          if (solver_response.contains("status")) {
            const auto& st = solver_response["status"];
            if (st.is_number_integer()) {
              status_code = st.get<int>();
              result->raw_status = std::to_string(status_code);
            } else if (st.is_string()) {
              result->raw_status = st.get<std::string>();
              status_code = (result->raw_status == "SUCCESS") ? 0 : -1;
            } else {
              result->raw_status = "UNKNOWN";
            }
          } else {
            result->raw_status = "UNKNOWN";
          }

          result->solution_cost = solver_response.value("solution_cost", 0.0);

          std::vector<std::pair<int, json>> vehicles;
          if (solver_response.contains("vehicle_data")) {
            for (auto& [vid_str, vdata] : solver_response["vehicle_data"].items()) {
              vehicles.emplace_back(std::stoi(vid_str), vdata);
            }
          }
          std::sort(vehicles.begin(), vehicles.end(),
                    [](const auto& a, const auto& b) { return a.first < b.first; });

          for (auto& [vid, vdata] : vehicles) {
            for (auto& loc : vdata.value("route", json::array())) {
              result->route_vehicle_ids.push_back(vid);
              result->route_locations.push_back(loc.get<int32_t>());
            }
          }

          result->success = (status_code == 0);
          if (!result->success) {
            result->error_message = "solver status: " + result->raw_status +
                                     " (0=SUCCESS,1=FAIL,2=TIMEOUT,3=EMPTY)";
          }
        } catch (const std::exception& e) {
          result->success = false;
          result->error_message = std::string("failed to parse solver response: ") + e.what();
        }

        // The ROS action itself succeeded in getting a definitive answer from the
        // server; whether the VRP solve was feasible is carried in result->success.
        goal_handle->succeed(result);
        return;
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    result->success = false;
    result->error_message = "timeout waiting for solution (reqId=" + req_id + ")";
    goal_handle->abort(result);
  }
};

int main(int argc, char** argv) {
  curl_global_init(CURL_GLOBAL_DEFAULT);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CuoptRoutingServerClient>());
  rclcpp::shutdown();
  curl_global_cleanup();
  return 0;
}
