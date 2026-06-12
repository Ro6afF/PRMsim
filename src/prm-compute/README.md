# 🧮 prm-compute

`prm-compute` is the high-performance Rust backend for the Interactive PRM Motion Planning Simulation. It is responsible for all heavy computations, including:

- **Forward Kinematics**: Calculating the position of the end-effector and joint links based on Denavit-Hartenberg (DH) parameters.
- **Collision Detection**: Checking robot configurations against spherical obstacles.
- **Sampling Strategies**: Generating valid joint configurations using Uniform, Gaussian, and Bridge sampling techniques.
- **Roadmap Construction**: Connecting sampled configurations to build a graph using strategies like K-Nearest or distance thresholds.
- **Pathfinding**: Using graph search algorithms (with customizable heuristics) to find a path from a start configuration to a target position.

This crate is compiled to WebAssembly (WASM) for use in the web frontend, but it also includes a native CLI tool for running automated benchmark sweeps.

## 📦 WebAssembly Build

When working on the web application, the crate is compiled to WASM using `wasm-pack`. 
This process is normally handled automatically by the frontend's build scripts (`npm run build:wasm`), which runs:

```bash
wasm-pack build --target web --out-dir ../wasm/prm-compute-pkg
```

## 📊 CLI Benchmark Tool

The `prm-compute` crate includes a standalone binary (`src/bin/benchmark.rs`) designed to stress-test and evaluate different PRM configurations. It uses Rayon to parallelize PRM generations and pathfinding attempts, enabling massive benchmark sweeps to be completed quickly.

### Running the Benchmark

You can run the benchmark natively using `cargo`:

```bash
cargo run --release --bin benchmark -- --config <path-to-toml-config> --output <path-to-results-csv>
```

#### Arguments
- `--config` (or `-c`): Path to a TOML file describing the benchmark sweeps.
- `--output` (or `-o`): The path for the output CSV file. Defaults to `results.csv`.

### Benchmark Configuration (TOML)

The benchmark is driven by a TOML configuration file that defines an array of `sweeps`. Each sweep specifies the parameters for roadmap generation and pathfinding.

Example configuration structure:

```toml
[[sweeps]]
name = "Gaussian Sampling Base Case"
prm_generations = 100
paths_per_prm = 50
robot_config_file = "../path/to/robot_dh.json"
obstacle_config_file = "../path/to/obstacles.json"

[sweeps.sampling_config]
num_samples = 250
sampling_strategy = "Gaussian"
sigma = 0.2

[sweeps.connection_config]
strategy = "KNearest"
k = 3
d = 0.1

[sweeps.planning_config]
heuristic = "joint-space"
num_restarts = 5
```

### Output Data

The benchmark evaluates the performance over the specified number of generations and writes the aggregated statistical results to a CSV file. The columns include:
- `name`: The name of the sweep.
- `median_build_time_ms` / `mean_build_time_ms`: Time taken to build the roadmap.
- `median_find_time_ms` / `mean_find_time_ms`: Time taken to find a path.
- `success_rate`: Percentage of successful pathfinding attempts.
- `median_edge_count` / `mean_edge_count`: Number of edges in the generated roadmap.
- `median_path_length` / `mean_path_length`: The length of successful paths in joint space.
