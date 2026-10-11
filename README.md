# cuopt_ros2

cuopt_ros2 integrates [NVIDIA cuOpt](https://docs.nvidia.com/cuopt/user-guide/latest/introduction.html), a GPU-accelerated optimization engine, into ROS2. It provides two independent packages, each wrapping a different cuOpt interface: a stable C API for LP/MILP/QP solving, and an internal C++ template API for vehicle routing (VRP). The purpose of this project is to make it easy for people to understand how to embed GPU-accelerated mathematical optimization and route planning inside the ROS2 ecosystem, including the build/link setup required to mix a CUDA-based solver library into a colcon workspace.

## System Requirements

- **OS**: Ubuntu 24.04
- **GPU**: NVIDIA GPU with Compute Capability >= 7.0 (Volta or newer)
- **CUDA**: 13.0
- **Driver**: NVIDIA driver compatible with CUDA 13.0
- **CMake**: >= 3.30.4 (only required to build `cuopt_routing_node`; see [Installation](#installation))

CPU/RAM/storage have not been formally benchmarked for this project — any workstation that can run ROS2 Jazzy and the CUDA 13 toolkit should be sufficient.

## Built With

- [ROS Jazzy](https://docs.ros.org/en/jazzy/) under Ubuntu 24.04 LTS
- [NVIDIA cuOpt](https://docs.nvidia.com/cuopt/user-guide/latest/introduction.html) (`libcuopt`)
- [RAPIDS RAFT](https://github.com/rapidsai/raft) / [RMM](https://github.com/rapidsai/rmm) (routing package only)

## Architecture

| Package | cuOpt interface | Purpose | Stability |
|---|---|---|---|
| [`cuopt_milp_node`](#cuopt_milp_node) | `cuopt_c.h` (`extern "C"` C API) | LP / MILP / QP solving, wrapped as a ROS2 Action Server | Officially supported, stable interface |
| [`cuopt_routing_node`](#cuopt_routing_node) | `cuopt::routing` C++ template API | Vehicle routing (VRP) demo, linked directly against the internal C++ classes in `libcuopt.so` | Internal implementation surface, no compatibility guarantee, PoC only |

Both packages link against the same `libcuopt.so`. `cuopt_routing_node` additionally requires RAFT/RMM headers, because the routing C++ API expects the caller to manage GPU device memory directly.

```
┌─────────────────────────────┐        ┌──────────────────────────────┐
│  cuopt_milp_node              │        │  cuopt_routing_node            │
│  (ament_cmake, CXX)           │        │  (ament_cmake, CXX + CUDA)      │
│                                │        │                                │
│  ActionServer "solve_milp"    │        │  cuopt_routing_demo (one-shot) │
│  -> cuOptCreateRangedProblem  │        │  -> raft::handle_t              │
│  -> cuOptSolve (host arrays)  │        │  -> rmm::device_uvector (GPU)   │
│  -> cuOptGetPrimalSolution    │        │  -> data_model_view_t / solve() │
└───────────────┬────────────────┘        └───────────────┬────────────────┘
                │ link -lcuopt                              │ link -lcuopt -lrmm -lraft
                ▼                                            ▼
                      libcuopt.so  (conda env: ~/miniconda3/envs/cuopt)
```

**Important**: the conda env (`~/miniconda3/envs/cuopt`) only provides headers and `.so` files. **Never `conda activate` it** when building or running the ROS2 workspace — it will conflict with ROS2's system Python. Everything is wired through the `CUOPT_SDK_PREFIX` environment variable and RPATH, so activation is never required.

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

### Build

```bash
cd ~/catkin_ws
source /opt/ros/jazzy/setup.bash
export CUOPT_SDK_PREFIX="$HOME/miniconda3/envs/cuopt"
export PATH="$HOME/.venvs/buildtools/bin:$PATH"

colcon build --packages-select cuopt_milp_node cuopt_routing_node
source install/setup.bash
```

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
  constraint_lower_bounds: [-1e30, -1e30],
  constraint_upper_bounds: [5.4, 4.9],
  var_lower_bounds: [0.0, 0.0],
  var_upper_bounds: [1e30, 1e30],
  is_integer: [false, false],
  time_limit: 10.0
}"
```

For MILP, set `is_integer` to `true` for the relevant variables; the rest of the flow is unchanged.

### `cuopt_routing_node`

**This is a proof-of-concept demo, not a production node.** It is currently a one-shot executable ([`cuopt_routing_demo.cu`](cuopt_routing_node/src/cuopt_routing_demo.cu)), not a long-running Action Server. It links directly against `cuopt::routing::data_model_view_t<int,float>` / `solve()` in `libcuopt.so` — an internal C++ template API (exported with `CUOPT_EXPORT`, but not documented as a supported external interface) rather than the stable `extern "C"` API.

```bash
ros2 run cuopt_routing_node cuopt_routing_demo
```

Key differences from the C API:

- `data_model_view_t` setters (`add_cost_matrix`, `set_order_locations`) expect **GPU device pointers**. The caller must copy data to device memory with `rmm::device_uvector` + `cudaMemcpyAsync` first; the C API handles this internally.
- Requires `raft::handle_t`, hence the extra `libraft-headers` / `librmm` dependency — their version must match what `libcuopt` was built against.
- Only `data_model_view_t<int, float>` is explicitly instantiated in `libcuopt.so`; any other template type fails at link time with `undefined reference`.

The demo hardcodes a toy 4-location VRP (1 depot + 3 stops, 1 vehicle, no time windows or capacity constraints — see lines 18-28). On success it prints:

```
[INFO] [cuopt_routing_demo]: total cost = <total_cost>, vehicles used = <n>
```

#### Known limitations

- No ROS2 interface (topic/service/action) to pass in problem data — it is hardcoded. Turning this into a usable node means wrapping it as an Action Server (following the `cuopt_milp_node` pattern), and verifying the device-pointer convention of each additional setter (time windows, capacity, etc.) before wiring it in.
- `result.get_route()` returns `rmm::device_uvector<int>&`; it is not yet copied back to host and printed — only total cost and vehicle count are shown.
- No GPU warmup; the first call's CUDA context initialization latency is noticeably higher.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `CMake Error ... find_path CUOPT_INCLUDE_DIR` | `CUOPT_SDK_PREFIX` not exported, or `libcuopt` not installed in the conda env | Check `echo $CUOPT_SDK_PREFIX` and `ls $CUOPT_SDK_PREFIX/include/cuopt` |
| `#error RMM requires LIBCUDACXX_ENABLE_EXPERIMENTAL_MEMORY_RESOURCE` / `CCCL version 3.1 or newer` | Manually added include paths instead of using `find_package(rmm CONFIG)` | Confirm `cuopt_routing_node/CMakeLists.txt` calls `find_package(rmm CONFIG REQUIRED)` and links `rmm::rmm` / `raft::raft`, not a bare `-lrmm` |
| `CMake 3.30.4 or higher is required` | System `cmake` (apt) is too old | Use the venv-installed newer `cmake` and put it first on `PATH` |
| `error while loading shared libraries: libcuopt.so` | RPATH not applied, or `CUOPT_SDK_PREFIX` differs between build and run | `ldd install/.../cuopt_milp_action_server \| grep cuopt` and confirm it resolves inside the conda env |
| `rosidl`/Python behaves oddly after colcon build | A conda env was activated in the same shell | Open a new shell, confirm `CONDA_PREFIX` is empty, then rebuild |
| Routing demo crashes with an illegal memory access | A host pointer was passed to a `data_model_view_t` setter | Confirm the data was copied to device memory via `rmm::device_uvector` + `cudaMemcpy` before the call |

## Next Steps

1. `cuopt_milp_node`: `execute()` currently detaches a new `std::thread` per request. Under load, switch to a thread pool or an `rclcpp::executors::MultiThreadedExecutor` with callback groups.
2. `cuopt_routing_node`: wrap it as an Action Server (mirroring `cuopt_milp_node`), and add the remaining constraint setters (time windows, capacity) after verifying each one's device-pointer convention against `$CUOPT_SDK_PREFIX/include/cuopt/routing/data_model_view.hpp`.
3. If the routing C++ template API's version coupling (RAFT/RMM must match the exact versions `libcuopt` was built against) becomes a maintenance burden, consider switching to a standalone `cuopt-server` (REST/gRPC) with the ROS2 node as a client, which removes the RAPIDS header/ABI dependency entirely.

## Reference

1. [NVIDIA cuOpt User Guide](https://docs.nvidia.com/cuopt/user-guide/latest/introduction.html)
2. [NVIDIA cuOpt API Reference](https://docs.nvidia.com/cuopt/user-guide/latest/api.html)
3. [cuOpt Examples](https://github.com/NVIDIA/cuopt-examples)
4. [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
5. [RAPIDS RAFT](https://github.com/rapidsai/raft)
6. [RAPIDS RMM](https://github.com/rapidsai/rmm)

---

*License/copyright placeholder — fill in your project's actual license and copyright holder here. Note that cuOpt itself is licensed under Apache-2.0; any redistribution of cuOpt headers or binaries must comply with that license.*
