import type { RobotConfiguration, DhParameter, Obstacle } from '../../types/kinematics';
import { get_end_effector_position } from '../../wasm/prm-compute-pkg/prm_compute';

/**
 * Interpolates between two robot configurations.
 */
export const interpolateConfig = (start: RobotConfiguration, end: RobotConfiguration, t: number): RobotConfiguration => {
  return start.map((s, i) => {
    const e = end[i];
    if (s.type === 'rotational') {
      let diff = e.theta - s.theta;
      const range = s.max - s.min;
      const isFullRange = range >= (2 * Math.PI - 0.01);
      if (isFullRange) {
        diff = Math.atan2(Math.sin(diff), Math.cos(diff));
      }
      return { ...s, theta: s.theta + diff * t };
    } else if (s.type === 'translational') {
      return { ...s, d: s.d + (e.d - s.d) * t };
    }
    return s;
  });
};

/**
 * Checks if a path between two configurations is collision-free by sampling.
 */
export const isPathCollisionFree = (q1: RobotConfiguration, q2: RobotConfiguration, obstacles: Obstacle[]): boolean => {
  const steps = 50;
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    const q = interpolateConfig(q1, q2, t);
    const res = (get_end_effector_position(q, obstacles) as any);
    if (res.collision) return false;
  }
  return true;
};

/**
 * Simple iterative Inverse Kinematics (Gradient Descent).
 * Finds a configuration that reaches the target Cartesian position.
 */
export const findIK = (
  target: { x: number, y: number, z: number }, 
  initialGuess: RobotConfiguration, 
  obstacles: Obstacle[]
): RobotConfiguration | null => {
  let currentConfig = JSON.parse(JSON.stringify(initialGuess));
  const stepSize = 0.5;
  const maxIterations = 500;
  const tolerance = 1.0; // 1mm tolerance

  for (let i = 0; i < maxIterations; i++) {
    const res = (get_end_effector_position(currentConfig, []) as any);
    const pos = res.ee_position;
    const dx = target.x - pos[0];
    const dy = target.y - pos[1];
    const dz = target.z - pos[2];
    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < tolerance) {
      // Final collision check
      const finalRes = (get_end_effector_position(currentConfig, obstacles) as any);
      return finalRes.collision ? null : currentConfig;
    }

    // Numeric Jacobian approximation
    const nextConfig = currentConfig.map((joint: DhParameter, jIdx: number) => {
      if (joint.type === 'fixed') return joint;

      const delta = 0.0001;
      const perturbed = JSON.parse(JSON.stringify(currentConfig));
      if (joint.type === 'rotational') perturbed[jIdx].theta += delta;
      else perturbed[jIdx].d += delta;

      const resP = (get_end_effector_position(perturbed, []) as any);
      const posP = resP.ee_position;
      
      const gradX = (posP[0] - pos[0]) / delta;
      const gradY = (posP[1] - pos[1]) / delta;
      const gradZ = (posP[2] - pos[2]) / delta;

      const step = (gradX * dx + gradY * dy + gradZ * dz) * stepSize;
      
      if (joint.type === 'rotational') {
        let newTheta = joint.theta + step;
        // Clamp to limits
        newTheta = Math.max(joint.min, Math.min(joint.max, newTheta));
        return { ...joint, theta: newTheta };
      } else {
        let newD = joint.d + step;
        newD = Math.max(joint.min, Math.min(joint.max, newD));
        return { ...joint, d: newD };
      }
    });

    currentConfig = nextConfig;
  }

  return null;
};
