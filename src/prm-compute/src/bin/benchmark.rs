
use clap::Parser;
use csv::Writer;
use prm_compute::connecting::{check_collision, joint_space_dist};
use prm_compute::prm::{build_prm, find_path_to_target};
use prm_compute::sampling::{generate_samples_internal, get_random_config};
use prm_compute::types::{DhParameter, Obstacle, RoadmapConfig, SamplingStrategy};
use rand::rngs::SmallRng;
use rand::SeedableRng;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::fs;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Path to the TOML benchmark configuration file
    #[arg(short, long)]
    config: String,

    /// Output CSV file
    #[arg(short, long, default_value = "results.csv")]
    output: String,
}

#[derive(Deserialize, Debug)]
struct BenchConfig {
    sweeps: Vec<Sweep>,
}

#[derive(Deserialize, Debug)]
struct Sweep {
    name: String,
    prm_generations: usize,
    paths_per_prm: usize,
    robot_config_file: String,
    obstacle_config_file: String,
    sampling_config: SamplingConfig,
    connection_config: RoadmapConfig,
    planning_config: PlanningConfig,
}

#[derive(Deserialize, Debug)]
struct SamplingConfig {
    num_samples: usize,
    sampling_strategy: SamplingStrategy,
    sigma: f64,
}

#[derive(Deserialize, Debug)]
struct PlanningConfig {
    heuristic: String,
    num_restarts: usize,
}

#[derive(Serialize)]
struct SweepResult {
    name: String,
    prm_generations: usize,
    paths_per_prm: usize,
    median_build_time_ms: f64,
    mean_build_time_ms: f64,
    median_find_time_ms: f64,
    mean_find_time_ms: f64,
    success_rate: f64,
    median_edge_count: f64,
    mean_edge_count: f64,
    median_path_length: f64,
    mean_path_length: f64,
}

fn calculate_median(values: &mut [f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mid = values.len() / 2;
    if values.len().is_multiple_of(2) {
        (values[mid - 1] + values[mid]) / 2.0
    } else {
        values[mid]
    }
}

fn main() {
    let args = Args::parse();

    let config_path = std::path::Path::new(&args.config);
    let base_dir = config_path.parent().unwrap_or(std::path::Path::new(""));

    let config_content = fs::read_to_string(&args.config).expect("Failed to read config file");
    let bench_config: BenchConfig = toml::from_str(&config_content).expect("Failed to parse TOML config");

    let mut csv_writer = Writer::from_path(&args.output).expect("Failed to create CSV writer");

    for sweep in bench_config.sweeps {
        println!("Running sweep: {}", sweep.name);

        let dh_path = base_dir.join(&sweep.robot_config_file);
        let dh_content = fs::read_to_string(&dh_path).expect("Failed to read DH file");
        let mut dh_params: Vec<DhParameter> = serde_json::from_str(&dh_content).expect("Failed to parse DH");
        for p in &mut dh_params { p.preprocess(); }

        let obs_path = base_dir.join(&sweep.obstacle_config_file);
        let obs_content = fs::read_to_string(&obs_path).expect("Failed to read Obstacles file");
        let obstacles: Vec<Obstacle> = serde_json::from_str(&obs_content).expect("Failed to parse Obstacles");

        let completed_generations = AtomicUsize::new(0);
        let ten_percent = (sweep.prm_generations / 10).max(1);

        let generation_results: Vec<_> = (0..sweep.prm_generations)
            .into_par_iter()
            .map(|_| {
                let start_build = Instant::now();
                let samples = generate_samples_internal(
                    &dh_params,
                    &obstacles,
                    sweep.sampling_config.num_samples,
                    &sweep.sampling_config.sampling_strategy,
                    sweep.sampling_config.sigma,
                );
                let prm_result = build_prm(&samples, &obstacles, &sweep.connection_config);
                let build_time = start_build.elapsed().as_secs_f64() * 1000.0;
                let edge_count: usize = prm_result.adjacency.iter().map(|v| v.len()).sum::<usize>() / 2;

                // Parallelize path finding attempts
                let path_results: Vec<_> = (0..sweep.paths_per_prm)
                    .into_par_iter()
                    .map(|_| {
                        let mut rng = SmallRng::from_entropy();
                        let mut points = Vec::new();
                        
                        let start_config = loop {
                            let config = get_random_config(&dh_params, &mut rng);
                            prm_compute::kinematics::compute_kinematics(&config, &mut points);
                            if !check_collision(&points, &obstacles) {
                                break config;
                            }
                        };
                        let (_goal_config, target_pos) = loop {
                            let config = get_random_config(&dh_params, &mut rng);
                            prm_compute::kinematics::compute_kinematics(&config, &mut points);
                            if !check_collision(&points, &obstacles) {
                                break (config, *points.last().unwrap());
                            }
                        };

                        let start_find = Instant::now();
                        let path_opt = find_path_to_target(
                            &start_config,
                            &samples,
                            &prm_result.adjacency,
                            target_pos,
                            &obstacles,
                            &sweep.planning_config.heuristic,
                            sweep.planning_config.num_restarts,
                            &sweep.connection_config,
                        ).ok();
                        let find_time = start_find.elapsed().as_secs_f64() * 1000.0;

                        let path_len = if let Some(ref p) = path_opt {
                            let mut dist = 0.0;
                            for i in 0..p.len().saturating_sub(1) {
                                dist += joint_space_dist(&p[i], &p[i+1]);
                            }
                            dist
                        } else {
                            0.0
                        };

                        (path_opt.is_some(), find_time, path_len)
                    })
                    .collect();

                let count = completed_generations.fetch_add(1, Ordering::Relaxed) + 1;
                if count % ten_percent == 0 || count == sweep.prm_generations {
                    println!("  Progress: {} / {} PRM generations completed ({}%)", count, sweep.prm_generations, (count * 100) / sweep.prm_generations);
                }

                (build_time, edge_count as f64, path_results)
            })
            .collect();

        let mut build_times = Vec::new();
        let mut edge_counts = Vec::new();
        let mut find_times = Vec::new();
        let mut path_lengths = Vec::new();
        let mut successes = 0;
        let mut total_attempts = 0;

        for (build_time, edge_count, path_results) in generation_results {
            build_times.push(build_time);
            edge_counts.push(edge_count);
            for (is_success, find_time, path_len) in path_results {
                total_attempts += 1;
                find_times.push(find_time);
                if is_success {
                    successes += 1;
                    path_lengths.push(path_len);
                }
            }
        }

        let mean_build = build_times.iter().sum::<f64>() / build_times.len().max(1) as f64;
        let median_build = calculate_median(&mut build_times);

        let mean_find = find_times.iter().sum::<f64>() / find_times.len().max(1) as f64;
        let median_find = calculate_median(&mut find_times);

        let mean_edges = edge_counts.iter().sum::<f64>() / edge_counts.len().max(1) as f64;
        let median_edges = calculate_median(&mut edge_counts);

        let median_length = calculate_median(&mut path_lengths);
        let mean_length = path_lengths.iter().sum::<f64>() / path_lengths.len().max(1) as f64;

        let result = SweepResult {
            name: sweep.name,
            prm_generations: sweep.prm_generations,
            paths_per_prm: sweep.paths_per_prm,
            median_build_time_ms: median_build,
            mean_build_time_ms: mean_build,
            median_find_time_ms: median_find,
            mean_find_time_ms: mean_find,
            success_rate: if total_attempts > 0 { successes as f64 / total_attempts as f64 } else { 0.0 },
            median_edge_count: median_edges,
            mean_edge_count: mean_edges,
            median_path_length: median_length,
            mean_path_length: mean_length,
        };

        csv_writer.serialize(result).expect("Failed to write CSV row");
        csv_writer.flush().expect("Failed to flush CSV");
    }

    csv_writer.flush().expect("Failed to flush CSV");
    println!("Benchmark completed! Results written to {}", args.output);
}
