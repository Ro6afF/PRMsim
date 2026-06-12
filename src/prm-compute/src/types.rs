use serde::{Deserialize, Serialize};
#[derive(Deserialize, Serialize, Clone, PartialEq)]
#[serde(rename_all = "lowercase")]
pub enum JointType {
    Rotational,
    Translational,
    Fixed,
}

#[derive(Deserialize, Serialize, Clone, PartialEq, Debug)]
#[serde(rename_all = "lowercase")]
pub enum SamplingStrategy {
    Uniform,
    Gaussian,
    Bridge,
}

#[derive(Deserialize, Serialize, Clone)]
pub struct DhParameter {
    pub r#type: JointType,
    pub theta: f64,
    pub alpha: f64,
    pub a: f64,
    pub d: f64,
    pub min: f64,
    pub max: f64,
    #[serde(skip, default)] pub cos_alpha: f64,
    #[serde(skip, default)] pub sin_alpha: f64,
    #[serde(skip, default)] pub is_continuous: bool,
}

impl DhParameter {
    pub fn preprocess(&mut self) {
        self.cos_alpha = self.alpha.cos();
        self.sin_alpha = self.alpha.sin();
        self.is_continuous = (self.max - self.min) >= (2.0 * std::f64::consts::PI - 0.001);
    }
}

#[derive(Deserialize)]
pub struct Obstacle {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub radius: f64,
}

#[derive(Serialize)]
pub struct KinematicsResult {
    pub ee_position: [f64; 3],
    pub collision: bool,
}

#[derive(Deserialize, Serialize, Clone)]
pub struct Sample {
    pub params: Vec<DhParameter>,
}

#[derive(Serialize)]
pub struct PrmResult {
    pub adjacency: Vec<Vec<usize>>,
}
#[derive(Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
pub struct RoadmapConfig {
    pub strategy: String,
    pub k: usize,
    pub d: f64,
    pub avoid_cycles: bool,
}
#[derive(Deserialize)]
pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
