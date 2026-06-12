use wasm_bindgen::prelude::*;
use serde_wasm_bindgen;
use crate::types::*;

#[wasm_bindgen]
pub fn build_prm(samples_val: JsValue, obstacles_val: JsValue, config_val: JsValue) -> JsValue {
    let mut samples: Vec<Sample> = serde_wasm_bindgen::from_value(samples_val).unwrap();
    for s in &mut samples { for p in &mut s.params { p.preprocess(); } }
    let obstacles: Vec<Obstacle> = serde_wasm_bindgen::from_value(obstacles_val).unwrap();
    let config: RoadmapConfig = serde_wasm_bindgen::from_value(config_val).unwrap();

    let result = crate::prm::build_prm(&samples, &obstacles, &config);
    serde_wasm_bindgen::to_value(&result).unwrap()
}

#[derive(serde::Serialize)]
pub struct PathPlanningResult {
    pub success: bool,
    pub path: Option<Vec<Vec<DhParameter>>>,
    pub error: Option<String>,
}

#[wasm_bindgen]
pub fn find_path_to_target(
    current_params_val: JsValue,
    samples_val: JsValue,
    adjacency_val: JsValue,
    target_pos_val: JsValue,
    obstacles_val: JsValue,
    heuristic_type: String,
    num_restarts: usize,
    config_val: JsValue,
) -> JsValue {
    let mut current_params: Vec<DhParameter> = serde_wasm_bindgen::from_value(current_params_val).unwrap();
    for p in &mut current_params { p.preprocess(); }
    let mut samples: Vec<Sample> = serde_wasm_bindgen::from_value(samples_val).unwrap();
    for s in &mut samples { for p in &mut s.params { p.preprocess(); } }
    let adjacency: Vec<Vec<usize>> = serde_wasm_bindgen::from_value(adjacency_val).unwrap();
    let target_pos_obj: Position3D = serde_wasm_bindgen::from_value(target_pos_val).unwrap();
    let target_pos = nalgebra::Point3::new(target_pos_obj.x, target_pos_obj.y, target_pos_obj.z);
    let obstacles: Vec<Obstacle> = serde_wasm_bindgen::from_value(obstacles_val).unwrap();
    let config: RoadmapConfig = serde_wasm_bindgen::from_value(config_val).unwrap();

    let result = match crate::prm::find_path_to_target(&current_params, &samples, &adjacency, target_pos, &obstacles, &heuristic_type, num_restarts, &config) {
        Ok(path) => PathPlanningResult {
            success: true,
            path: Some(path),
            error: None,
        },
        Err(err) => PathPlanningResult {
            success: false,
            path: None,
            error: Some(err),
        },
    };
    serde_wasm_bindgen::to_value(&result).unwrap()
}

#[wasm_bindgen]
pub fn get_end_effector_position(params: JsValue, obstacles: JsValue) -> JsValue {
    let mut dh_params: Vec<DhParameter> = serde_wasm_bindgen::from_value(params).unwrap();
    for p in &mut dh_params { p.preprocess(); }
    let obstacles: Vec<Obstacle> = serde_wasm_bindgen::from_value(obstacles).unwrap();

    let mut points = Vec::new();
    crate::kinematics::compute_kinematics(&dh_params, &mut points);
    let ee_pos = points.last().unwrap();
    let collision = crate::connecting::check_collision(&points, &obstacles);

    let result = KinematicsResult {
        ee_position: [ee_pos.x, ee_pos.y, ee_pos.z],
        collision,
    };

    serde_wasm_bindgen::to_value(&result).unwrap()
}

#[wasm_bindgen]
pub fn generate_samples(
    current_params_val: JsValue,
    obstacles_val: JsValue,
    num_samples: usize,
    strategy_val: JsValue,
    sigma: f64,
) -> JsValue {
    let mut dh_params: Vec<DhParameter> = serde_wasm_bindgen::from_value(current_params_val).unwrap();
    for p in &mut dh_params { p.preprocess(); }
    let obstacles: Vec<Obstacle> = serde_wasm_bindgen::from_value(obstacles_val).unwrap();
    let strategy: SamplingStrategy = serde_wasm_bindgen::from_value(strategy_val).unwrap_or(SamplingStrategy::Uniform);

    let samples = crate::sampling::generate_samples_internal(&dh_params, &obstacles, num_samples, &strategy, sigma);
    serde_wasm_bindgen::to_value(&samples).unwrap()
}
