#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#if __has_include(<cuopt/mathematical_optimization/cuopt_c.h>)
  #include <cuopt/mathematical_optimization/cuopt_c.h>
  #include <cuopt/mathematical_optimization/constants.h>
#else
  #include <cuopt/linear_programming/cuopt_c.h>
  #include <cuopt/linear_programming/constants.h>
#endif
#include "cuopt_milp_node/action/solve_milp.hpp"

using SolveMilp = cuopt_milp_node::action::SolveMilp;
using GoalHandle = rclcpp_action::ServerGoalHandle<SolveMilp>;

class CuoptMilpActionServer : public rclcpp::Node {
public:
  CuoptMilpActionServer() : Node("cuopt_milp_action_server") {
    server_ = rclcpp_action::create_server<SolveMilp>(
      this, "solve_milp",
      std::bind(&CuoptMilpActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CuoptMilpActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&CuoptMilpActionServer::handle_accepted, this, std::placeholders::_1));
    warmup();  // GPU/CUDA context 暖機,避免第一個真正的 request 卡住
  }

private:
  rclcpp_action::Server<SolveMilp>::SharedPtr server_;

  void warmup() {
    cuopt_int_t ro[] = {0, 1};
    cuopt_int_t ci[] = {0};
    cuopt_float_t v[] = {1.0};
    cuopt_float_t obj[] = {1.0};
    cuopt_float_t clb[] = {-CUOPT_INFINITY}, cub[] = {1.0};
    cuopt_float_t vlb[] = {0.0}, vub[] = {CUOPT_INFINITY};
    char vt[] = {CUOPT_CONTINUOUS};
    cuOptOptimizationProblem p = nullptr;
    cuOptCreateRangedProblem(1, 1, CUOPT_MINIMIZE, 0.0, obj, ro, ci, v, clb, cub, vlb, vub, vt, &p);
    cuOptSolverSettings s = nullptr;
    cuOptCreateSolverSettings(&s);
    cuOptSolution sol = nullptr;
    cuOptSolve(p, s, &sol);
    cuOptDestroySolution(&sol);
    cuOptDestroySolverSettings(&s);
    cuOptDestroyProblem(&p);
    RCLCPP_INFO(get_logger(), "cuOpt GPU warmup done");
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                           std::shared_ptr<const SolveMilp::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void handle_accepted(const std::shared_ptr<GoalHandle> gh) {
    std::thread{[this, gh] { execute(gh); }}.detach();  // 避免卡住 executor
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle) {
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<SolveMilp::Result>();

    std::vector<char> var_types(goal->is_integer.size());
    for (size_t i = 0; i < goal->is_integer.size(); ++i)
      var_types[i] = goal->is_integer[i] ? CUOPT_INTEGER : CUOPT_CONTINUOUS;

    cuOptOptimizationProblem problem = nullptr;
    cuopt_int_t status = cuOptCreateRangedProblem(
      goal->num_constraints, goal->num_variables, CUOPT_MINIMIZE, 0.0,
      goal->objective_coefficients.data(),
      goal->row_offsets.data(), goal->col_indices.data(), goal->matrix_values.data(),
      goal->constraint_lower_bounds.data(), goal->constraint_upper_bounds.data(),
      goal->var_lower_bounds.data(), goal->var_upper_bounds.data(),
      var_types.data(), &problem);

    if (status != CUOPT_SUCCESS) {
      result->success = false;
      result->error_message = "cuOptCreateRangedProblem failed";
      goal_handle->abort(result);
      return;
    }

    cuOptSolverSettings settings = nullptr;
    cuOptCreateSolverSettings(&settings);
    cuOptSetFloatParameter(settings, CUOPT_TIME_LIMIT, goal->time_limit);

    cuOptSolution solution = nullptr;
    status = cuOptSolve(problem, settings, &solution);

    if (status == CUOPT_SUCCESS) {
      cuopt_float_t obj;
      cuOptGetObjectiveValue(solution, &obj);
      result->objective_value = obj;
      result->solution.resize(goal->num_variables);
      cuOptGetPrimalSolution(solution, result->solution.data());
      result->success = true;
      goal_handle->succeed(result);
    } else {
      result->success = false;
      result->error_message = "cuOptSolve failed";
      goal_handle->abort(result);
    }

    cuOptDestroySolution(&solution);
    cuOptDestroySolverSettings(&settings);
    cuOptDestroyProblem(&problem);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CuoptMilpActionServer>());
  rclcpp::shutdown();
  return 0;
}