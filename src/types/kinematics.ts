export type JointType = 'rotational' | 'translational' | 'fixed';

export interface DhParameter {
  type: JointType;
  theta: number;
  alpha: number;
  a: number;
  d: number;
  min: number;
  max: number;
}

export type RobotConfiguration = DhParameter[];

export interface KinematicsResult {
  ee_position: [number, number, number];
  collision: boolean;
}

export interface Obstacle {
  x: number;
  y: number;
  z: number;
  radius: number;
}

export interface Sample {
  params: DhParameter[];
}

export type SamplingStrategy = 'uniform' | 'gaussian' | 'bridge';

export type RoadmapConnectingStrategy = 'k-nearest' | 'normalized-distance';

export interface RoadmapConfig {
  strategy: RoadmapConnectingStrategy;
  k: number;
  d: number;
  avoidCycles: boolean;
}

export type Roadmap = number[][];
