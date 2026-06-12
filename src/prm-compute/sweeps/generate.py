def generate_sweep():
    toml = ""
    def add_sweep(name, sampling_strategy="uniform", sigma=0.05, num_samples=100, 
                  conn_strategy="k-nearest", k=3, d=0.2, avoidCycles="false",
                  heuristic="none"):
        nonlocal toml
        toml += "[[sweeps]]\n"
        toml += f'name = "{name}"\n'
        toml += "prm_generations = 1000\n"
        toml += "paths_per_prm = 1000\n"
        toml += 'robot_config_file = "robot.json"\n'
        toml += 'obstacle_config_file = "obstacles.json"\n'
        toml += "[sweeps.sampling_config]\n"
        toml += f"num_samples = {num_samples}\n"
        toml += f'sampling_strategy = "{sampling_strategy}"\n'
        if sampling_strategy in ["gaussian", "bridge"]:
            toml += f"sigma = {sigma}\n"
        else:
            toml += "sigma = 0.05\n" # Default for others
        toml += "[sweeps.connection_config]\n"
        toml += f'strategy = "{conn_strategy}"\n'
        toml += f"k = {k}\n"
        toml += f"d = {d}\n"
        toml += f"avoidCycles = {avoidCycles}\n"
        toml += "[sweeps.planning_config]\n"
        toml += f'heuristic = "{heuristic}"\n'
        toml += "num_restarts = 100\n\n"

    toml += "# --- Baselines ---\n\n"
    add_sweep("Baseline K-Nearest", conn_strategy="k-nearest")
    add_sweep("Baseline Distance", conn_strategy="distance")

    toml += "# --- Impact of Connectivity ---\n\n"
    for k in [2, 4]:
        add_sweep(f"Connectivity K-Nearest (k={k})", conn_strategy="k-nearest", k=k)
    for d in [0.175, 0.225]:
        add_sweep(f"Connectivity Distance (d={d})", conn_strategy="distance", d=d)

    toml += "# --- Impact of Pathfinding Heuristic ---\n\n"
    add_sweep("Heuristic Joint-Space (K-Nearest)", conn_strategy="k-nearest", heuristic="joint-space")
    add_sweep("Heuristic Joint-Space (Distance)", conn_strategy="distance", heuristic="joint-space")

    toml += "# --- Impact of Cycle Avoidance ---\n\n"
    add_sweep("Cycle Avoidance True (K-Nearest)", conn_strategy="k-nearest", avoidCycles="true")
    add_sweep("Cycle Avoidance True (Distance)", conn_strategy="distance", avoidCycles="true")

    toml += "# --- Impact of Sampling Density ---\n\n"
    for num_samples in [50, 200]:
        add_sweep(f"Density {num_samples} (K-Nearest)", num_samples=num_samples, conn_strategy="k-nearest")
        add_sweep(f"Density {num_samples} (Distance)", num_samples=num_samples, conn_strategy="distance")

    toml += "# --- Impact of Sampling Strategy ---\n\n"
    for sig in [0.04, 0.05, 0.06]:
        add_sweep(f"Sampling Gaussian (sigma={sig}) (K-Nearest)", sampling_strategy="gaussian", sigma=sig, conn_strategy="k-nearest")
        add_sweep(f"Sampling Gaussian (sigma={sig}) (Distance)", sampling_strategy="gaussian", sigma=sig, conn_strategy="distance")
    for sig in [0.04, 0.05, 0.06]:
        add_sweep(f"Sampling Bridge (sigma={sig}) (K-Nearest)", sampling_strategy="bridge", sigma=sig, conn_strategy="k-nearest")
        add_sweep(f"Sampling Bridge (sigma={sig}) (Distance)", sampling_strategy="bridge", sigma=sig, conn_strategy="distance")

    return toml

base_toml = generate_sweep()

import os

# Write individual sweeps
envs = ["canopy", "choke_point", "clutter", "three_hubs"]
for env in envs:
    with open(f"{env}/sweep.toml", "w") as f: 
        f.write(base_toml)

# Write combined sweep
with open("combined_sweep.toml", "w") as f:
    for env in envs:
        f.write(f"# === Sweeps from {env} ===\n\n")
        lines = base_toml.split('\n')
        for line in lines:
            if line.startswith('name = '):
                f.write(f'name = "[{env}] {line[8:-1]}"\n')
            elif line.startswith('robot_config_file = '):
                f.write(f'robot_config_file = "{env}/robot.json"\n')
            elif line.startswith('obstacle_config_file = '):
                f.write(f'obstacle_config_file = "{env}/obstacles.json"\n')
            else:
                f.write(line + '\n')
