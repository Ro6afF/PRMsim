use crate::types::*;
use crate::connecting::*;
use crate::sampling::*;
use nalgebra::{DMatrix, DVector, Matrix3, Point3, Vector3};
use rand::rngs::SmallRng;
use rand::SeedableRng;
pub fn compute_kinematics(dh_params: &[DhParameter], points: &mut Vec<Point3<f64>>) {
    let mut current_rot = Matrix3::identity();
    let mut current_pos = Vector3::zeros();
    points.clear();
    points.push(Point3::from(current_pos));

    for p in dh_params {
        let (st, ct) = p.theta.sin_cos();
        let ca = p.cos_alpha;
        let sa = p.sin_alpha;

        // Step 1 & 2: Rz(theta) and Tz(d)
        let t1 = Vector3::new(0.0, 0.0, p.d);
        let pos1 = current_pos + current_rot * t1;
        points.push(Point3::from(pos1));
        
        #[rustfmt::skip]
        let r1 = Matrix3::new(
            ct, -st, 0.0,
            st,  ct, 0.0,
            0.0, 0.0, 1.0,
        );
        let rot1 = current_rot * r1;

        // Step 3 & 4: Tx(a) and Rx(alpha)
        let t2 = Vector3::new(p.a, 0.0, 0.0);
        let pos2 = pos1 + rot1 * t2;
        points.push(Point3::from(pos2));

        #[rustfmt::skip]
        let r2 = Matrix3::new(
            1.0, 0.0, 0.0,
            0.0,  ca, -sa,
            0.0,  sa,  ca,
        );
        current_rot = rot1 * r2;
        current_pos = pos2;
    }
}
pub fn compute_jacobian(params: &mut [DhParameter]) -> DMatrix<f64> {
    let delta = 0.0001;
    let n_joints = params.len();
    let mut jacobian = DMatrix::zeros(3, n_joints); // Can be changed to 6xN later

    let mut points = Vec::new();
    compute_kinematics(params, &mut points);
    let base_ee = *points.last().unwrap();

    for j in 0..n_joints {
        if params[j].r#type == JointType::Fixed {
            continue;
        }

        let old_val = if params[j].r#type == JointType::Rotational {
            let v = params[j].theta;
            params[j].theta += delta;
            v
        } else {
            let v = params[j].d;
            params[j].d += delta;
            v
        };

        compute_kinematics(params, &mut points);
        let perturbed_ee = points.last().unwrap();

        let diff = perturbed_ee - base_ee;
        
        jacobian[(0, j)] = diff.x / delta;
        jacobian[(1, j)] = diff.y / delta;
        jacobian[(2, j)] = diff.z / delta;
        
        // For orientation, compute angular difference and add to rows 3,4,5
        
        if params[j].r#type == JointType::Rotational {
            params[j].theta = old_val;
        } else {
            params[j].d = old_val;
        }
    }

    jacobian
}

pub fn solve_ik_dls(
    start_params: &[DhParameter],
    target_pos: Point3<f64>,
    max_iters: usize,
    tolerance: f64,
) -> Option<Vec<DhParameter>> {
    let mut current_params = start_params.to_vec();
    let lambda_sq = 0.01; // Damping factor (lambda = 0.1)

    let mut points = Vec::new();
    let identity = DMatrix::identity(3, 3);
    let mut error_col = DVector::zeros(3);

    for _ in 0..max_iters {
        compute_kinematics(&current_params, &mut points);
        let current_pos = points.last().unwrap();
        
        let error_vec = target_pos - current_pos;
        let error_norm = error_vec.norm();
        
        if error_norm < tolerance {
            return Some(current_params);
        }

        let jacobian = compute_jacobian(&mut current_params);
        let j_t = jacobian.transpose();
        
        let j_jt = &jacobian * &j_t;
        let damped = j_jt + &identity * lambda_sq;
        
        let inv_damped = damped.try_inverse()?;

        error_col[0] = error_vec.x;
        error_col[1] = error_vec.y;
        error_col[2] = error_vec.z;
        let delta_theta = &j_t * inv_damped * &error_col;

        for j in 0..current_params.len() {
            if current_params[j].r#type == JointType::Fixed {
                continue;
            }
            
            if current_params[j].r#type == JointType::Rotational {
                current_params[j].theta += delta_theta[j];
                // Clamp or wrap
                let range = current_params[j].max - current_params[j].min;
                let is_full_range = range >= (2.0 * std::f64::consts::PI - 0.001);
                if is_full_range {
                    let mut val = current_params[j].theta;
                    while val > current_params[j].max { val -= 2.0 * std::f64::consts::PI; }
                    while val < current_params[j].min { val += 2.0 * std::f64::consts::PI; }
                    current_params[j].theta = val;
                } else {
                    current_params[j].theta = current_params[j].theta.clamp(current_params[j].min, current_params[j].max);
                }
            } else if current_params[j].r#type == JointType::Translational {
                current_params[j].d += delta_theta[j];
                current_params[j].d = current_params[j].d.clamp(current_params[j].min, current_params[j].max);
            }
        }
    }
    
    // Check one last time
    compute_kinematics(&current_params, &mut points);
    let current_pos = points.last().unwrap();
    if (target_pos - current_pos).norm() < tolerance {
        Some(current_params)
    } else {
        None
    }
}

pub fn find_multiple_ik_solutions(
    template_params: &[DhParameter],
    obstacles: &[Obstacle],
    target_pos: Point3<f64>,
    num_restarts: usize,
) -> Vec<Vec<DhParameter>> {
    let mut solutions: Vec<Vec<DhParameter>> = Vec::new();
    let mut rng = SmallRng::from_entropy();
    let mut points = Vec::new();

    for _ in 0..num_restarts {
        let seed = get_random_config(template_params, &mut rng);
        if let Some(solution) = solve_ik_dls(&seed, target_pos, 100, 1.0) {
            compute_kinematics(&solution, &mut points);
            if !check_collision(&points, obstacles) {
                // Check distinct
                let is_distinct = solutions.iter().all(|s| joint_space_dist(&solution, s) > 0.05);
                if is_distinct {
                    solutions.push(solution);
                }
            }
        }
    }
    
    solutions
}
