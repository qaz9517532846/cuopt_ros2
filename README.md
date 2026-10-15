# cuopt_ros2

cuopt_ros2 integrates [NVIDIA cuOpt](https://docs.nvidia.com/cuopt/user-guide/latest/introduction.html), a GPU-accelerated optimization engine, into ROS2. It provides three independent packages, each wrapping a different cuOpt interface: a stable C API for LP/MILP/QP solving, an internal C++ template API for vehicle routing (VRP) linked in-process, and an HTTP client against a standalone `cuopt-server` for VRP that keeps ROS2 fully decoupled from CUDA/RAPIDS. The purpose of this project is to make it easy for people to understand how to embed GPU-accelerated mathematical optimization and route planning inside the ROS2 ecosystem, including the build/link setup required to mix a CUDA-based solver library into a colcon workspace, and the trade-offs between linking it in-process versus calling it over the network.

## System Requirements

- **OS**: Ubuntu 24.04
- **GPU**: NVIDIA GPU with Compute Capability >= 7.0 (Volta or newer)
- **CUDA**: 13.0
- **Driver**: NVIDIA driver compatible with CUDA 13.0
- **CMake**: >= 3.30.4 (only required to build `cuopt_routing_node`; see [Installation](#installation))

CPU/RAM/storage have not been formally benchmarked for this project — any workstation that can run ROS2 Jazzy and the CUDA 13 toolkit should be sufficient.

## Built With

- [ROS Jazzy](https://docs.ros.org/en/jazzy/) under Ubuntu 24.04 LTS
- [NVIDIA cuOpt](https://docs.nvidia.com/cuopt/user-guide/latest/introduction.html) (`libcuopt`, and the `cuopt-server` Docker image)
- [RAPIDS RAFT](https://github.com/rapidsai/raft) / [RMM](https://github.com/rapidsai/rmm) (`cuopt_routing_node` only)
- [libcurl](https://curl.se/libcurl/) / [nlohmann/json](https://github.com/nlohmann/json) (`cuopt_routing_server_client` only)

## Architecture

| Package | cuOpt interface | Purpose | Stability |
|---|---|---|---|
| [`cuopt_milp_node`](#cuopt_milp_node) | `cuopt_c.h` (`extern "C"` C API) | LP / MILP / QP solving, wrapped as a ROS2 Action Server | Officially supported, stable interface |
| [`cuopt_routing_node`](#cuopt_routing_node) | `cuopt::routing` C++ template API | Vehicle routing (VRP) demo, linked directly against the internal C++ classes in `libcuopt.so` | Internal implementation surface, no compatibility guarantee, PoC only |
| [`cuopt_routing_server_client`](#cuopt_routing_server_client) | `cuopt-server` REST API (`/cuopt/request`, `/cuopt/solution/{id}`) | Vehicle routing (VRP), wrapped as a ROS2 Action Server that talks HTTP to a standalone `cuopt-server` container | Officially supported server API; this package has no CUDA/RAPIDS dependency at all |

`cuopt_milp_node` and `cuopt_routing_node` link against the same `libcuopt.so`; `cuopt_routing_node` additionally requires RAFT/RMM headers, because the routing C++ API expects the caller to manage GPU device memory directly. `cuopt_routing_server_client` does not link `libcuopt.so` at all — it only speaks HTTP to a `cuopt-server` process (typically a Docker container), which is the whole point of that package: zero RAPIDS header/ABI coupling in the ROS2 workspace, at the cost of a network hop and a separately-run server process.

```
┌─────────────────────────┐  ┌──────────────────────────┐  ┌──────────────────────────────┐
│ cuopt_milp_node          │  │ cuopt_routing_node         │  │ cuopt_routing_server_client     │
│ (ament_cmake, CXX)       │  │ (ament_cmake, CXX+CUDA)    │  │ (ament_cmake, CXX)             │
│                           │  │                            │  │                                │
│ ActionServer "solve_milp"│  │ cuopt_routing_demo         │  │ ActionServer "solve_vrp"        │
│ -> cuOptCreateRangedProb.│  │ (one-shot)                 │  │ -> HTTP POST /cuopt/request     │
│ -> cuOptSolve (host arr.)│  │ -> raft::handle_t          │  │ -> poll GET /cuopt/solution/{id}│
│ -> cuOptGetPrimalSolution│  │ -> rmm::device_uvector(GPU)│  │ (libcurl + nlohmann/json)       │
└──────────────┬─────────────┘ -> data_model_view_t/solve()└─────────────────┬────────────────┘
               │ link -lcuopt   └──────────────┬─────────────┘                │ HTTP (localhost:8000)
               ▼                               │ link -lcuopt -lrmm -lraft    ▼
        libcuopt.so  <────────────────────────┘                    cuopt-server container
  (conda env: ~/miniconda3/envs/cuopt)                          (docker run --gpus all ...)
```

**Important**: the conda env (`~/miniconda3/envs/cuopt`) only provides headers and `.so` files, and is only needed by `cuopt_milp_node` / `cuopt_routing_node`. **Never `conda activate` it** when building or running the ROS2 workspace — it will conflict with ROS2's system Python. Everything is wired through the `CUOPT_SDK_PREFIX` environment variable and RPATH, so activation is never required. `cuopt_routing_server_client` needs none of this — only `libcurl4-openssl-dev` and `nlohmann-json3-dev` (both standard apt packages).

## Getting Started

### Installation

1. **Install the cuOpt SDK (headers + shared library) into a conda env**

   ```bash
   conda create -n cuopt python=3.12
   conda install -n cuopt -c rapidsai -c conda-forge -c nvidia \
     libcuopt libraft-headers librmm cuda-nvcc cuda-version=13
   ```

   Do **not** activate it. Just record the path:

   ```bash
   export CUOPT_SDK_PREFIX="$HOME/miniconda3/envs/cuopt"
   ```

   > The header layout depends on the installed `libcuopt` version: older releases ship `cuopt/linear_programming/cuopt_c.h`, newer releases rename it to `cuopt/mathematical_optimization/cuopt_c.h`. Both packages already handle this with `find_path(NAMES ...)` / `__has_include`, so upgrading `libcuopt` does not require source changes.

2. **Install ROS2 Jazzy** (if not already installed)

   ```bash
   sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions
   ```

3. **Install a CMake >= 3.30.4** (only needed for `cuopt_routing_node` — RMM/RAFT's CMake config files require it, and Ubuntu 24.04's apt package is 3.28.3)

   ```bash
   python3 -m venv ~/.venvs/buildtools
   ~/.venvs/buildtools/bin/pip install "cmake>=3.30.4"
   export PATH="$HOME/.venvs/buildtools/bin:$PATH"
   cmake --version   # confirm >= 3.30.4
   ```

   Recommended `~/.bashrc` additions (keep this order):

   ```bash
   source /opt/ros/jazzy/setup.bash
   export CUOPT_SDK_PREFIX="$HOME/miniconda3/envs/cuopt"
   export PATH="$HOME/.venvs/buildtools/bin:$PATH"
   ```

4. **`cuopt_routing_server_client` dependencies** (no conda/CMake-version requirement — only system packages)

   ```bash
   sudo apt install libcurl4-openssl-dev nlohmann-json3-dev
   ```

5. **Pull and run the `cuopt-server` container** (only needed for `cuopt_routing_server_client`)

   ```bash
   docker pull nvidia/cuopt:latest-cu13
   docker run --gpus all -d -p 8000:8000 -e CUOPT_SERVER_PORT=8000 --name cuopt-server nvidia/cuopt:latest-cu13
   curl http://localhost:8000/cuopt/health   # -> {"status":"RUNNING","version":"..."}
   ```

   The `CUOPT_SERVER_PORT=8000` env var is required — the container's internal Uvicorn server listens on port **5000** by default regardless of the `-p` mapping, so without it `-p 8000:8000` maps to nothing and every request gets connection-refused. (Confirmed by testing: `docker logs cuopt-server` without the env var shows `Uvicorn running on http://0.0.0.0:5000`.)

   Requires the NVIDIA Container Toolkit (`docker info | grep -i runtime` should list `nvidia`). The server has no authentication — only expose port 8000 on a trusted network.

### Build

```bash
cd ~/catkin_ws
source /opt/ros/jazzy/setup.bash
export CUOPT_SDK_PREFIX="$HOME/miniconda3/envs/cuopt"
export PATH="$HOME/.venvs/buildtools/bin:$PATH"

colcon build --packages-select cuopt_milp_node cuopt_routing_node cuopt_routing_server_client
source install/setup.bash
```

`cuopt_routing_server_client` has no dependency on `CUOPT_SDK_PREFIX`, conda, or the upgraded CMake — it can be built on its own with just `colcon build --packages-select cuopt_routing_server_client`.

Verify conda is not active in the build shell:

```bash
echo "CONDA_PREFIX=$CONDA_PREFIX"   # must be empty
```

## Run

### `cuopt_milp_node`

Wraps the `cuopt_c.h` call sequence (`cuOptCreateRangedProblem` -> `cuOptSolve` -> `cuOptGetPrimalSolution`) as a ROS2 **Action Server**. An action is used instead of a service because solve time is not fixed and callers may need progress feedback or cancellation.

```bash
ros2 run cuopt_milp_node cuopt_milp_action_server
```

On startup the node solves a tiny 1-variable/1-constraint LP as a GPU warmup. The log line `cuOpt GPU warmup done` confirms `libcuopt.so` and the CUDA context loaded correctly.

#### Action interface: `solve_milp`

Defined in [`cuopt_milp_node/action/SolveMilp.action`](cuopt_milp_node/action/SolveMilp.action), mirroring the CSR-format arguments of `cuOptCreateRangedProblem`:

| Field | Type | Description |
|---|---|---|
| `num_variables` / `num_constraints` | `int32` | Problem size |
| `objective_coefficients` | `float64[]` | Linear objective coefficients |
| `row_offsets` / `col_indices` / `matrix_values` | `int32[]` / `int32[]` / `float64[]` | CSR-format constraint matrix |
| `constraint_lower_bounds` / `constraint_upper_bounds` | `float64[]` | Per-constraint bounds |
| `var_lower_bounds` / `var_upper_bounds` | `float64[]` | Per-variable bounds |
| `is_integer` | `bool[]` | `true` -> `CUOPT_INTEGER`, `false` -> `CUOPT_CONTINUOUS` |
| `time_limit` | `float64` | Solver time limit (seconds) |
| **Result** `success` | `bool` | Whether the solve succeeded |
| **Result** `objective_value` | `float64` | Final objective value |
| **Result** `solution` | `float64[]` | Primal variable values |
| **Result** `error_message` | `string` | Populated on failure |
| **Feedback** `status` | `string` | Progress string |

The objective sense is currently fixed to `CUOPT_MINIMIZE` (see [`cuopt_milp_action_server.cpp`](cuopt_milp_node/src/cuopt_milp_action_server.cpp) line 70); adding a `MAXIMIZE` option requires a new Goal field passed through to `cuOptCreateRangedProblem`.

#### Example call

Solve `minimize -0.2*x1 + 0.1*x2` subject to `3*x1 + 4*x2 <= 5.4`, `2.7*x1 + 10.1*x2 <= 4.9`, `x1, x2 >= 0`:

```bash
ros2 action send_goal /solve_milp cuopt_milp_node/action/SolveMilp "{
  num_variables: 2,
  num_constraints: 2,
  objective_coefficients: [-0.2, 0.1],
  row_offsets: [0, 2, 4],
  col_indices: [0, 1, 0, 1],
  matrix_values: [3.0, 4.0, 2.7, 10.1],
  constraint_lower_bounds: [-1.0e+30, -1.0e+30],
  constraint_upper_bounds: [5.4, 4.9],
  var_lower_bounds: [0.0, 0.0],
  var_upper_bounds: [1.0e+30, 1.0e+30],
  is_integer: [false, false],
  time_limit: 10.0
}"
```

For MILP, set `is_integer` to `true` for the relevant variables; the rest of the flow is unchanged.

### `cuopt_routing_node`

**These are proof-of-concept demos, not a production node.** There are three one-shot executables, not a long-running Action Server, each mirroring one of the official cuOpt C++ routing examples (`cpp/tests/examples/routing/`). All link directly against `cuopt::routing::data_model_view_t<int,float>` / `solve()` in `libcuopt.so` — an internal C++ template API (exported with `CUOPT_EXPORT`, but not documented as a supported external interface) rather than the stable `extern "C"` API.

Key differences from the C API, shared by all three:

- `data_model_view_t` setters (`add_cost_matrix`, `set_order_locations`, `add_capacity_dimension`, `set_order_time_windows`, ...) expect **GPU device pointers**. The caller must copy data to device memory first — either manually with `rmm::device_uvector` + `cudaMemcpyAsync`, or with the vendored `cuopt::device_copy()` helper (see below). The C API handles this internally instead.
- Requires `raft::handle_t`, hence the extra `libraft-headers` / `librmm` dependency — their version must match what `libcuopt` was built against.
- Only `data_model_view_t<int, float>` is explicitly instantiated in `libcuopt.so`; any other template type fails at link time with `undefined reference`.

#### 1. `cuopt_routing_demo` — CVRPTW

Modeled after `cvrp_daily_deliveries.cu` + `service_team_routing.cu`: depot (0) + 5 delivery points, 2 vehicles, one capacity dimension, generous time windows.

```bash
ros2 run cuopt_routing_node cuopt_routing_demo
```
```
total cost = 130.000000, vehicles used = 2
vehicle 0 route: 0 1 5 2 0
vehicle 1 route: 0 3 4 0
```
(verified on this GPU — all 5 order locations covered exactly once, demand 90+60=150 split under two 100-unit-capacity vehicles)

#### 2. `pdptw_mixed_fleet_demo` — mixed fleet + pickup-delivery

Modeled after `pdptw_mixed_fleet.cu`: depot + 4 pickup/delivery points, 2 small trucks (capacity 100) + 1 large truck (capacity 200), 2 pickup-delivery pairs (demand ±50 and ±150 — the ±150 pair can only ride the large truck).

```bash
ros2 run cuopt_routing_node pdptw_mixed_fleet_demo
```
```
total cost = 85.000000, vehicles used = 1
vehicle 2 (type=1, large truck) route: 0 1 3 4 2 0
```
(verified on this GPU — the solver put both pairs on the single large truck rather than splitting across 2 vehicles, since minimizing vehicle count is the default primary objective; cost 85 matches the sum of that exact edge sequence in the cost matrix)

#### 3. `service_team_routing_demo` — time windows + breaks

Modeled after `service_team_routing.cu`: depot + 4 service points, 3 technician teams, each with a mandatory break taken at the depot.

```bash
ros2 run cuopt_routing_node service_team_routing_demo
```
```
total cost = 140.000000, teams used = 3
team 0 route: 0(type=0) 1(type=2) 0(type=3) 3(type=2) 0(type=0)
team 1 route: 0(type=0) 0(type=3) 2(type=2) 0(type=0)
team 2 route: 0(type=0) 0(type=3) 4(type=2) 0(type=0)
```
(`type=0`=depot, `type=2`=service call, `type=3`=break — this installed version doesn't expose the enum's names, so the demo prints the raw int)

#### Vendored header

[`include/cuopt_routing_node/copy_helpers.hpp`](cuopt_routing_node/include/cuopt_routing_node/copy_helpers.hpp) is copied verbatim (Apache-2.0, copyright notice preserved) from cuOpt's own `cpp/src/utilities/copy_helpers.hpp`. It provides the `cuopt::device_copy(std::vector<T>, stream)` convenience used by all three demos for every host->device transfer. It is **not** shipped in the installed `libcuopt` package (it lives under `cpp/src/`, not `cpp/include/`), which is why it has to be vendored rather than included from `$CUOPT_SDK_PREFIX`. The output side needs no vendoring: `host_assignment_t<int>` (route/truck_id/locations/etc., device->host copy included) is part of the public, installed `cuopt/routing/assignment.hpp`.

#### API gotchas found while building these (each verified against a real solve, not just read from docs)

1. **`add_capacity_dimension`'s `demand` array is sized by order count, not location count.** The header's doc comment says the `demand` pointer should cover "each location, including the depot" (i.e. `num_locations` entries) — but the official `cvrp_daily_deliveries.cu` example passes one entry per *order* (`num_orders` entries, no depot slot), and that's what actually works here too. Trust the tested example over the docstring.
2. **`host_assignment_t::route` is not what it sounds like for non-depot stops.** Despite `assignment_t::get_route()`'s doc comment saying it returns "a vector of locations", cross-checking against `node_types` and `locations` on a real solve showed `route[i]` holds the **order index** (e.g. `route=0` at a delivery stop was order index 0, i.e. `h_order_locations[0]`) wherever `node_types[i]` is a delivery/pickup, not the physical location. **`host_assignment_t::locations`** is the field that gives the actual location id at every stop, depot included — use that to print or act on a route, not `route`.
3. **`set_break_locations` takes a *set* of eligible locations shared across all vehicles, not one entry per vehicle.** The official `service_team_routing.cu` example passes `{0, 0, 0}` (one zero per of its 3 vehicles) in its comment's spirit of "one break location per team" — but this installed `libcuopt` throws `cuopt::logic_error`: `"There should be unique break locations"` on that input, since it's duplicate entries in what is actually a deduplicated eligible-location set. Fixed here by passing `{0}` (depot is the only eligible break location, shared by every vehicle) — this crashed the whole process (uncaught exception on a detached thread would do the same in an Action Server; see [Troubleshooting](#troubleshooting) for why that matters).
4. **`add_cost_matrix` is keyed by `vehicle_type`, and every declared type needs its own call once `set_vehicle_types()` is used.** `add_cost_matrix(matrix, vehicle_type = 0)` defaults to type 0; calling `set_vehicle_types()` with more than one type (e.g. `{0, 0, 1}` for a mixed fleet) but only ever calling `add_cost_matrix()` once leaves type 1 without a matrix, and the solver rejects the model with `"All vehicle cost matrices should be set"` — a validation error, not a crash, but still a confusingly generic top-level message (`get_status_string()` just says "An error occured while running the cuOpt solver."; the real reason is in `get_error_status().what()`, not surfaced anywhere else). Fixed in `pdptw_mixed_fleet_demo` by calling `add_cost_matrix(v_cost_matrix.data(), 0)` and `add_cost_matrix(v_cost_matrix.data(), 1)` with the same matrix (all vehicle types share one road network/distances here).

#### Known limitations

- No ROS2 interface (topic/service/action) to pass in problem data in any of the three — it is hardcoded. Turning these into usable nodes means wrapping them as Action Servers (following the `cuopt_milp_node` pattern).
- No GPU warmup in any of the three; the first call's CUDA context initialization latency is noticeably higher.
- `pdptw_mixed_fleet_demo` and `service_team_routing_demo` do not catch exceptions from `cuopt::logic_error` (e.g. the break-location validation error above) — a malformed data model currently aborts the whole process rather than failing gracefully. Fine for a one-shot demo; would need a try/catch around `solve()` in any Action Server wrapper.

### `cuopt_routing_server_client`

A production-shaped alternative to `cuopt_routing_node`: a long-running **Action Server** that solves VRP by calling a standalone `cuopt-server` over HTTP, instead of linking `libcuopt.so` in-process. This package has no CUDA/RAPIDS dependency — only `libcurl` and `nlohmann/json`.

```bash
ros2 run cuopt_routing_server_client cuopt_routing_action_server --ros-args -p server_url:=http://localhost:8000
```

`server_url` defaults to `http://localhost:8000`. On startup the node does a non-blocking health check against `/cuopt/health` and logs a warning (not a crash) if the server isn't reachable yet — it will still accept goals and retry the HTTP call when one arrives.

#### Action interface: `solve_vrp`

Defined in [`cuopt_routing_server_client/action/SolveVrp.action`](cuopt_routing_server_client/action/SolveVrp.action). Mirrors the `cuopt-server` REST payload (`cost_matrix_data` / `task_data` / `fleet_data` / `solver_config`) for a capacitated VRP with time windows and a single capacity dimension:

| Field | Type | Description |
|---|---|---|
| `num_locations` | `int32` | Total locations including the depot (index 0) |
| `cost_matrix` / `travel_time_matrix` | `float64[]` | Flattened row-major `num_locations x num_locations`; leave `travel_time_matrix` empty to reuse `cost_matrix` |
| `task_locations` | `int32[]` | Location index per task (depot excluded) |
| `demand` | `int32[]` | Demand per task, single capacity dimension |
| `task_time_windows_earliest` / `_latest` | `float64[]` | Per-task time window |
| `service_times` | `float64[]` | Per-task service time |
| `vehicle_start_locations` / `vehicle_end_locations` | `int32[]` | Per-vehicle start/end location |
| `vehicle_capacities` | `int32[]` | Per-vehicle capacity, single dimension |
| `vehicle_time_windows_earliest` / `_latest` | `float64[]` | Per-vehicle time window |
| `solver_time_limit` | `float64` | Passed as `solver_config.time_limit` |
| `server_url` | `string` | Optional per-goal override of the `server_url` parameter |
| **Result** `success` | `bool` | `true` only if the server's `solver_response.status == 0` |
| **Result** `raw_status` | `string` | `solver_response.status` stringified — the server sends it as a **number** (`0`=SUCCESS, `1`=FAIL, `2`=TIMEOUT, `3`=EMPTY), not the string the name might suggest |
| **Result** `solution_cost` | `float64` | Total route cost |
| **Result** `route_vehicle_ids` / `route_locations` | `int32[]` | Parallel arrays: flattened routes, grouped by vehicle, in order |
| **Result** `error_message` | `string` | Populated on failure (HTTP error, timeout, or infeasible) |
| **Feedback** `status` | `string` | Submission/polling progress |

#### Example call

Same 5-location VRP with time windows as the cuOpt server skill's reference example (1 depot + 4 tasks, 2 vehicles, capacity 100 each):

```bash
ros2 action send_goal /solve_vrp cuopt_routing_server_client/action/SolveVrp "{
  num_locations: 5,
  cost_matrix: [0,10,15,20,25, 10,0,12,18,22, 15,12,0,10,15, 20,18,10,0,8, 25,22,15,8,0],
  task_locations: [1, 2, 3, 4],
  demand: [20, 30, 25, 15],
  task_time_windows_earliest: [0, 10, 20, 0],
  task_time_windows_latest: [50, 60, 70, 80],
  service_times: [5, 5, 5, 5],
  vehicle_start_locations: [0, 0],
  vehicle_end_locations: [0, 0],
  vehicle_capacities: [100, 100],
  vehicle_time_windows_earliest: [0, 0],
  vehicle_time_windows_latest: [200, 200],
  solver_time_limit: 10.0
}"
```

Verified end-to-end against a real `nvidia/cuopt:latest-cu13` container; the actual result for the goal above is:

```
Result:
    success: true
    raw_status: '0'
    solution_cost: 65.0
    route_vehicle_ids: [0, 0, 0, 0, 0, 0]
    route_locations: [0, 1, 2, 3, 4, 0]
    error_message: ''
```

Read the two result arrays pairwise (`route_vehicle_ids[i]` owns stop `route_locations[i]`) to reconstruct each vehicle's route — here, a single vehicle visits depot -> 1 -> 2 -> 3 -> 4 -> depot.

#### Known limitations

- Single capacity dimension only; multi-dimension capacity, pickup-delivery pairs, and precedence constraints are not exposed in the action and would need new fields plumbed into the JSON payload.
- Polls once per second with no backoff; fine for demo-scale problems, but a production version should use an exponential backoff and a configurable poll interval.
- No TLS/auth — matches the `cuopt-server` itself, which ships with neither. Keep it on a trusted network, as noted above.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 action send_goal` fails with `Failed to populate message fields: TypeError('must be real number, not str')` | PyYAML only parses scientific notation as a float when it has **both** a decimal point and a signed exponent (`1.0e+30`); `1e30` or `1.0e30` are parsed as the *string* `'1e30'` and fail to fit a `float64` field | Always write large bounds as `1.0e+30` / `-1.0e+30`, never bare `1e30` |
| `CMake Error ... find_path CUOPT_INCLUDE_DIR` | `CUOPT_SDK_PREFIX` not exported, or `libcuopt` not installed in the conda env | Check `echo $CUOPT_SDK_PREFIX` and `ls $CUOPT_SDK_PREFIX/include/cuopt` |
| `#error RMM requires LIBCUDACXX_ENABLE_EXPERIMENTAL_MEMORY_RESOURCE` / `CCCL version 3.1 or newer` | Manually added include paths instead of using `find_package(rmm CONFIG)` | Confirm `cuopt_routing_node/CMakeLists.txt` calls `find_package(rmm CONFIG REQUIRED)` and links `rmm::rmm` / `raft::raft`, not a bare `-lrmm` |
| `CMake 3.30.4 or higher is required` | System `cmake` (apt) is too old | Use the venv-installed newer `cmake` and put it first on `PATH` |
| `error while loading shared libraries: libcuopt.so` | RPATH not applied, or `CUOPT_SDK_PREFIX` differs between build and run | `ldd install/.../cuopt_milp_action_server \| grep cuopt` and confirm it resolves inside the conda env |
| `rosidl`/Python behaves oddly after colcon build | A conda env was activated in the same shell | Open a new shell, confirm `CONDA_PREFIX` is empty, then rebuild |
| Routing demo crashes with an illegal memory access | A host pointer was passed to a `data_model_view_t` setter | Confirm the data was copied to device memory via `rmm::device_uvector` + `cudaMemcpy` before the call |
| `cuopt_routing_server_client` logs "cuopt-server not reachable" | Container not running, still starting up, or wrong `server_url` | `docker ps` to confirm the container is up; `curl http://localhost:8000/cuopt/health`; the node still accepts goals and will retry at call time |
| Health check / goal connection refused even though the container is `Up` | Missing `-e CUOPT_SERVER_PORT=8000` — the container listens on port 5000 internally by default | `docker logs <container>` and check the `Uvicorn running on http://0.0.0.0:<port>` line; re-run with `-e CUOPT_SERVER_PORT=8000` |
| `solve_vrp` goal result has `success: false, error_message: "submit failed: ..."` | cuopt-server rejected the JSON payload (HTTP 422) | Check array lengths line up (e.g. `cost_matrix` size must equal `num_locations^2`, `capacities` is nested as one array per vehicle in the raw REST schema — this package already handles that nesting for you) |
| Docker container exits immediately | No GPU visible to the container | Run with `--gpus all` and confirm `docker info` lists the `nvidia` runtime |

## Next Steps

1. `cuopt_milp_node`: `execute()` currently detaches a new `std::thread` per request. Under load, switch to a thread pool or an `rclcpp::executors::MultiThreadedExecutor` with callback groups. The same applies to `cuopt_routing_server_client`.
2. `cuopt_routing_node`: wrap each of the three demos as an Action Server (mirroring `cuopt_milp_node`), and add a try/catch around `solve()` so a malformed data model returns a failed goal instead of aborting the process.
3. `cuopt_routing_server_client`: extend the action/JSON mapping for multi-dimension capacities, pickup-delivery pairs, and precedence constraints; add exponential backoff to the poll loop.
4. If the routing use case is the primary one going forward, `cuopt_routing_server_client` is the lower-maintenance choice over `cuopt_routing_node` — it has no RAFT/RMM version coupling to `libcuopt`, at the cost of a network hop and a separately-managed server process.

## Reference

1. [NVIDIA cuOpt User Guide](https://docs.nvidia.com/cuopt/user-guide/latest/introduction.html)
2. [NVIDIA cuOpt API Reference](https://docs.nvidia.com/cuopt/user-guide/latest/api.html)
3. [cuOpt Examples](https://github.com/NVIDIA/cuopt-examples)
4. [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
5. [RAPIDS RAFT](https://github.com/rapidsai/raft)
6. [RAPIDS RMM](https://github.com/rapidsai/rmm)

---

*This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2026 ZM Robotics Software Laboratory.*
