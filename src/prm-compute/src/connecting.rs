use crate::types::*;
use crate::kinematics::*;
use nalgebra::Point3;
pub fn joint_space_dist(params1: &[DhParameter], params2: &[DhParameter]) -> f64 {
    let mut sum_sq = 0.0;
    for (p1, p2) in params1.iter().zip(params2.iter()) {
        match p1.r#type {
            JointType::Rotational => {
                let mut diff = p1.theta - p2.theta;
                if p1.is_continuous {
                    while diff > std::f64::consts::PI { diff -= 2.0 * std::f64::consts::PI; }
                    while diff < -std::f64::consts::PI { diff += 2.0 * std::f64::consts::PI; }
                }
                let range = p1.max - p1.min;
                let norm_diff = if range > 0.0 { diff / range } else { 0.0 };
                sum_sq += norm_diff * norm_diff;
            }
            JointType::Translational => {
                let diff = p1.d - p2.d;
                let range = p1.max - p1.min;
                let norm_diff = if range > 0.0 { diff / range } else { 0.0 };
                sum_sq += norm_diff * norm_diff;
            }
            JointType::Fixed => {}
        }
    }
    sum_sq.sqrt()
}

pub fn check_path_collision(
    start_params: &[DhParameter],
    end_params: &[DhParameter],
    obstacles: &[Obstacle],
    steps: usize,
) -> bool {
    let mut points = Vec::new();
    let mut interp_params = start_params.to_vec();
    for step in 1..steps {
        let t = step as f64 / steps as f64;
        for (i, p_start) in start_params.iter().enumerate() {
            let p_end = &end_params[i];
            match p_start.r#type {
                JointType::Rotational => {
                    let mut diff = p_end.theta - p_start.theta;
                    if p_start.is_continuous {
                        while diff > std::f64::consts::PI { diff -= 2.0 * std::f64::consts::PI; }
                        while diff < -std::f64::consts::PI { diff += 2.0 * std::f64::consts::PI; }
                    }
                    let mut new_theta = p_start.theta + diff * t;
                    if p_start.is_continuous {
                        if new_theta > p_start.max {
                            new_theta -= 2.0 * std::f64::consts::PI;
                        } else if new_theta < p_start.min {
                            new_theta += 2.0 * std::f64::consts::PI;
                        }
                    }
                    interp_params[i].theta = new_theta;
                }
                JointType::Translational => {
                    interp_params[i].d = p_start.d + (p_end.d - p_start.d) * t;
                }
                JointType::Fixed => {}
            }
        }

        compute_kinematics(&interp_params, &mut points);
        if check_collision(&points, obstacles) {
            return true;
        }
    }
    false
}
fn dist_to_segment_squared(point: Point3<f64>, segment: (Point3<f64>, Point3<f64>)) -> f64 {
    let segment_vec = segment.1 - segment.0;
    let center_vec = point - segment.0;

    let segment_len_sq = segment_vec.norm_squared();
    if segment_len_sq < 1e-9 {
        return center_vec.norm_squared();
    }

    let proj = center_vec.dot(&segment_vec) / segment_len_sq;
    let proj_clamped = proj.clamp(0.0, 1.0);

    let closest = segment.0 + (segment_vec * proj_clamped);

    (closest - point).norm_squared()
}

pub fn check_collision(points: &[Point3<f64>], obstacles: &[Obstacle]) -> bool {
    for i in 0..(points.len().saturating_sub(1)) {
        let p1 = points[i];
        let p2 = points[i + 1];
        
        let min_x = p1.x.min(p2.x);
        let max_x = p1.x.max(p2.x);
        let min_y = p1.y.min(p2.y);
        let max_y = p1.y.max(p2.y);
        let min_z = p1.z.min(p2.z);
        let max_z = p1.z.max(p2.z);

        for obstacle in obstacles {
            if obstacle.x + obstacle.radius < min_x || obstacle.x - obstacle.radius > max_x ||
               obstacle.y + obstacle.radius < min_y || obstacle.y - obstacle.radius > max_y ||
               obstacle.z + obstacle.radius < min_z || obstacle.z - obstacle.radius > max_z {
                continue;
            }

            let center = Point3::new(obstacle.x, obstacle.y, obstacle.z);
            let radius_sq = obstacle.radius * obstacle.radius;

            if dist_to_segment_squared(center, (p1, p2)) < radius_sq {
                return true;
            }
        }
    }

    false
}
