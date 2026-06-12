use crate::types::*;
use crate::kinematics::*;
use crate::connecting::*;
use nalgebra::Point3;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

struct Dsu {
    parent: Vec<usize>,
}

impl Dsu {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    fn find(&mut self, i: usize) -> usize {
        if self.parent[i] == i {
            i
        } else {
            self.parent[i] = self.find(self.parent[i]);
            self.parent[i]
        }
    }

    fn union(&mut self, i: usize, j: usize) -> bool {
        let root_i = self.find(i);
        let root_j = self.find(j);
        if root_i != root_j {
            self.parent[root_i] = root_j;
            true
        } else {
            false
        }
    }
}

pub fn build_prm(samples: &[Sample], obstacles: &[Obstacle], config: &RoadmapConfig) -> PrmResult {
    let n = samples.len();
    let mut adjacency = vec![Vec::new(); n];
    let mut dsu = Dsu::new(n);
    let mut connected_pairs = HashSet::new();

    if n == 0 {
        return PrmResult { adjacency };
    }

    let active_params: Vec<DhParameter> = samples[0].params.iter().filter(|p| p.r#type != JointType::Fixed).cloned().collect();
    let dim = active_params.len();
    
    let mut kdtree = kdtree::KdTree::new(dim);
    let mut points = Vec::with_capacity(n);
    for (i, sample) in samples.iter().enumerate() {
        let pt: Vec<f64> = sample.params.iter().filter(|p| p.r#type != JointType::Fixed).map(|p| {
            if p.r#type == JointType::Rotational { p.theta } else { p.d }
        }).collect();
        kdtree.add(pt.clone(), i).unwrap();
        points.push(pt);
    }

    let dist_fn = |a: &[f64], b: &[f64]| -> f64 {
        let mut sum_sq = 0.0;
        for (i, p) in active_params.iter().enumerate() {
            let mut diff = a[i] - b[i];
            if p.r#type == JointType::Rotational && p.is_continuous {
                while diff > std::f64::consts::PI { diff -= 2.0 * std::f64::consts::PI; }
                while diff < -std::f64::consts::PI { diff += 2.0 * std::f64::consts::PI; }
            }
            let range = p.max - p.min;
            let norm_diff = if range > 0.0 { diff / range } else { 0.0 };
            sum_sq += norm_diff * norm_diff;
        }
        sum_sq
    };

    for i in 0..n {
        if config.strategy == "k-nearest" {
            if let Ok(nearest) = kdtree.nearest(&points[i], config.k + 1, &dist_fn) {
                for &(_dist_sq, &j) in &nearest {
                    if i == j { continue; }
                    try_connect(i, j, samples, obstacles, config.avoid_cycles, &mut dsu, &mut adjacency, &mut connected_pairs);
                }
            }
        } else {
            if let Ok(within) = kdtree.within(&points[i], config.d * config.d, &dist_fn) {
                let mut within: Vec<_> = within;
                within.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
                for &(_dist_sq, &j) in &within {
                    if i == j { continue; }
                    try_connect(i, j, samples, obstacles, config.avoid_cycles, &mut dsu, &mut adjacency, &mut connected_pairs);
                }
            }
        }
    }

    PrmResult { adjacency }
}

fn try_connect(
    i: usize,
    j: usize,
    samples: &[Sample],
    obstacles: &[Obstacle],
    avoid_cycles: bool,
    dsu: &mut Dsu,
    adjacency: &mut Vec<Vec<usize>>,
    connected_pairs: &mut HashSet<(usize, usize)>,
) {
    let pair = if i < j { (i, j) } else { (j, i) };
    if connected_pairs.contains(&pair) {
        return;
    }

    if avoid_cycles && dsu.find(i) == dsu.find(j) {
        return;
    }

    if !check_path_collision(&samples[i].params, &samples[j].params, obstacles, 100) {
        connected_pairs.insert(pair);
        adjacency[i].push(j);
        adjacency[j].push(i);
        if avoid_cycles {
            dsu.union(i, j);
        }
    }
}

#[derive(Copy, Clone)]
struct State {
    cost: f64,
    position: usize,
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.position == other.position
    }
}

impl Eq for State {}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.partial_cmp(&self.cost).unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn get_temp_connections(
    temp_sample: &Sample,
    samples: &[Sample],
    obstacles: &[Obstacle],
    config: &RoadmapConfig,
) -> Vec<usize> {
    let mut potential_neighbors = Vec::new();
    for (j, sample) in samples.iter().enumerate() {
        let dist = joint_space_dist(&temp_sample.params, &sample.params);
        if config.strategy == "k-nearest" {
            potential_neighbors.push((dist, j));
        } else if (config.strategy == "normalized-distance" || config.strategy == "distance")
            && dist <= config.d {
                potential_neighbors.push((dist, j));
            }
    }

    if config.strategy == "k-nearest" {
        if potential_neighbors.len() > config.k {
            potential_neighbors.select_nth_unstable_by(config.k, |a, b| a.0.partial_cmp(&b.0).unwrap());
            potential_neighbors.truncate(config.k);
        }
        potential_neighbors.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    } else {
        potential_neighbors.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    }

    let mut connections = Vec::new();
    for (_dist, j) in potential_neighbors {
        if !check_path_collision(&temp_sample.params, &samples[j].params, obstacles, 100) {
            connections.push(j);
        }
    }
    connections
}

pub fn find_path_to_target(
    current_params: &[DhParameter],
    samples: &[Sample],
    adjacency: &[Vec<usize>],
    target_pos: Point3<f64>,
    obstacles: &[Obstacle],
    heuristic_type: &str,
    num_restarts: usize,
    config: &RoadmapConfig,
) -> Result<Vec<Vec<DhParameter>>, String> {
    let mut points = Vec::new();
    compute_kinematics(current_params, &mut points);
    if check_collision(&points, obstacles) {
        return Err("The robot's current configuration is in collision with an obstacle. Please move the robot or remove obstacles before planning.".to_string());
    }

    let n = samples.len();
    
    let mut temp_samples = Vec::new();
    let mut temp_adjacency = Vec::new();
    let mut extra_incoming_edges: HashMap<usize, Vec<usize>> = HashMap::new();

    // 1. Start node
    let start_idx = n;
    let start_sample = Sample { params: current_params.to_vec() };
    let start_connections = get_temp_connections(&start_sample, samples, obstacles, config);
    if start_connections.is_empty() {
        return Err("The robot's current configuration could not be connected to the roadmap. Try generating more samples or increasing connection distance.".to_string());
    }
    for &j in &start_connections {
        extra_incoming_edges.entry(j).or_default().push(start_idx);
    }
    temp_samples.push(start_sample);
    temp_adjacency.push(start_connections);

    // 2. Target nodes
    let ik_solutions = find_multiple_ik_solutions(current_params, obstacles, target_pos, num_restarts);
    if ik_solutions.is_empty() {
        return Err("Could not find a collision-free joint configuration (IK solution) for the target position. The target is either out of the robot's physical reach or blocked by obstacles.".to_string());
    }

    let mut target_indices = Vec::new();
    let mut total_target_connections = 0;
    for solution in ik_solutions {
        let t_idx = n + temp_samples.len();
        let t_sample = Sample { params: solution };
        let t_connections = get_temp_connections(&t_sample, samples, obstacles, config);
        total_target_connections += t_connections.len();
        for &j in &t_connections {
            extra_incoming_edges.entry(j).or_default().push(t_idx);
        }
        temp_samples.push(t_sample);
        temp_adjacency.push(t_connections);
        target_indices.push(t_idx);
    }

    if total_target_connections == 0 {
        return Err("The target configuration could not be connected to the roadmap. Try generating more samples or increasing connection distance.".to_string());
    }

    // 3. Virtual goal node
    let virtual_goal_idx = n + temp_samples.len();
    temp_samples.push(Sample { params: current_params.to_vec() }); // Dummy params
    temp_adjacency.push(Vec::new());
    
    for &t_idx in &target_indices {
        temp_adjacency[t_idx - n].push(virtual_goal_idx);
    }

    let get_sample = |idx: usize| -> &Sample {
        if idx < n {
            &samples[idx]
        } else {
            &temp_samples[idx - n]
        }
    };

    let mut heap = BinaryHeap::new();
    let mut came_from: HashMap<usize, usize> = HashMap::new();
    let mut g_score: HashMap<usize, f64> = HashMap::new();

    g_score.insert(start_idx, 0.0);

    let calc_heuristic = |curr_idx: usize| -> f64 {
        if heuristic_type == "dijkstra" || curr_idx == virtual_goal_idx {
            0.0
        } else {
            target_indices.iter()
                .map(|&t_idx| joint_space_dist(&get_sample(curr_idx).params, &get_sample(t_idx).params))
                .fold(f64::INFINITY, f64::min)
        }
    };

    heap.push(State {
        cost: calc_heuristic(start_idx),
        position: start_idx,
    });

    while let Some(State { cost: _, position: current }) = heap.pop() {
        if current == virtual_goal_idx {
            let mut path_indices = Vec::new();
            let mut curr = *came_from.get(&virtual_goal_idx).unwrap();
            path_indices.push(curr);
            while let Some(&prev) = came_from.get(&curr) {
                path_indices.push(prev);
                curr = prev;
            }
            path_indices.reverse();
            
            let full_path: Vec<Vec<DhParameter>> = path_indices.into_iter().map(|idx| get_sample(idx).params.clone()).collect();
            return Ok(full_path);
        }

        let current_g = *g_score.get(&current).unwrap();

        let mut evaluate_neighbor = |neighbor: usize| {
            let weight = if neighbor == virtual_goal_idx {
                0.0
            } else {
                joint_space_dist(&get_sample(current).params, &get_sample(neighbor).params)
            };
            
            let tentative_g_score = current_g + weight;
            let neighbor_g = g_score.get(&neighbor).copied().unwrap_or(f64::INFINITY);

            if tentative_g_score < neighbor_g {
                came_from.insert(neighbor, current);
                g_score.insert(neighbor, tentative_g_score);
                let h = calc_heuristic(neighbor);
                heap.push(State {
                    cost: tentative_g_score + h,
                    position: neighbor,
                });
            }
        };

        if current < n {
            for &neighbor in &adjacency[current] {
                evaluate_neighbor(neighbor);
            }
            if let Some(extra) = extra_incoming_edges.get(&current) {
                for &neighbor in extra {
                    evaluate_neighbor(neighbor);
                }
            }
        } else {
            for &neighbor in &temp_adjacency[current - n] {
                evaluate_neighbor(neighbor);
            }
        }
    }

    Err("No path exists between the current configuration and the target in the current roadmap. Try generating more samples or increasing the connection distance to connect the roadmap components.".to_string())
}
