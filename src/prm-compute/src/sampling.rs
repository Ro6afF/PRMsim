use crate::types::*;
use crate::kinematics::*;
use crate::connecting::*;
use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};
use rand_distr::{Distribution, Normal};

pub fn get_random_config(template: &[DhParameter], rng: &mut SmallRng) -> Vec<DhParameter> {
    let mut random_params = template.to_vec();
    for p in &mut random_params {
        match p.r#type {
            JointType::Rotational => {
                p.theta = rng.gen_range(p.min..=p.max);
            }
            JointType::Translational => {
                p.d = rng.gen_range(p.min..=p.max);
            }
            JointType::Fixed => {}
        }
    }
    random_params
}

pub fn generate_samples_internal(
    dh_params: &[DhParameter],
    obstacles: &[Obstacle],
    num_samples: usize,
    strategy: &SamplingStrategy,
    sigma: f64,
) -> Vec<Sample> {
    let mut samples = Vec::new();
    let mut rng = SmallRng::from_entropy();
    let mut points = Vec::new();

    while samples.len() < num_samples {
        match strategy {
            SamplingStrategy::Uniform => {
                let random_params = get_random_config(dh_params, &mut rng);
                compute_kinematics(&random_params, &mut points);
                if !check_collision(&points, obstacles) {
                    samples.push(Sample {
                        params: random_params,
                    });
                }
            }
            SamplingStrategy::Gaussian => {
                let q1 = get_random_config(dh_params, &mut rng);
                compute_kinematics(&q1, &mut points);
                let coll1 = check_collision(&points, obstacles);

                let mut q2 = q1.clone();
                for p in &mut q2 {
                    let range = (p.max - p.min).abs();
                    if range > 0.0 {
                        let normal = Normal::new(0.0, range * sigma).unwrap();
                        match p.r#type {
                            JointType::Rotational => {
                                p.theta = (p.theta + normal.sample(&mut rng)).clamp(p.min, p.max);
                            }
                            JointType::Translational => {
                                p.d = (p.d + normal.sample(&mut rng)).clamp(p.min, p.max);
                            }
                            JointType::Fixed => {}
                        }
                    }
                }

                compute_kinematics(&q2, &mut points);
                let coll2 = check_collision(&points, obstacles);

                if coll1 != coll2 {
                    let free_config = if !coll1 { q1 } else { q2 };
                    samples.push(Sample {
                        params: free_config,
                    });
                }
            }
            SamplingStrategy::Bridge => {
                let q1 = get_random_config(dh_params, &mut rng);
                compute_kinematics(&q1, &mut points);
                if !check_collision(&points, obstacles) {
                    continue;
                }

                let mut q2 = q1.clone();
                for p in &mut q2 {
                    let range = (p.max - p.min).abs();
                    if range > 0.0 {
                        let normal = Normal::new(0.0, range * sigma).unwrap();
                        match p.r#type {
                            JointType::Rotational => {
                                p.theta = (p.theta + normal.sample(&mut rng)).clamp(p.min, p.max);
                            }
                            JointType::Translational => {
                                p.d = (p.d + normal.sample(&mut rng)).clamp(p.min, p.max);
                            }
                            JointType::Fixed => {}
                        }
                    }
                }

                compute_kinematics(&q2, &mut points);
                if !check_collision(&points, obstacles) {
                    continue;
                }

                let mut qm = q1.clone();
                for (i, p) in qm.iter_mut().enumerate() {
                    match p.r#type {
                        JointType::Rotational => {
                            p.theta = (q1[i].theta + q2[i].theta) / 2.0;
                        }
                        JointType::Translational => {
                            p.d = (q1[i].d + q2[i].d) / 2.0;
                        }
                        JointType::Fixed => {}
                    }
                }

                compute_kinematics(&qm, &mut points);
                if !check_collision(&points, obstacles) {
                    samples.push(Sample {
                        params: qm,
                    });
                }
            }
        }
    }
    samples
}
