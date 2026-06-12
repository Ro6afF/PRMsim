import React, { useMemo, useRef, useEffect, useState, useCallback } from 'react';
import * as THREE from 'three';
import { Canvas, useThree, useFrame } from '@react-three/fiber';
import { OrbitControls, TransformControls, Line } from '@react-three/drei';
import { get_end_effector_position } from '../wasm/prm-compute-pkg/prm_compute';
import type { RobotConfiguration, Obstacle, KinematicsResult, Sample, Roadmap } from '../types/kinematics';
import { interpolateConfig } from '../utils/kinematics';

interface Props {
  config: RobotConfiguration;
  obstacles: Obstacle[];
  onObstaclesChange?: (obstacles: Obstacle[]) => void;
  selectedObstacleIndex?: number | null;
  onObstacleSelect?: (index: number | null) => void;
  collision?: boolean;
  initialViewState?: { position: [number, number, number], target: [number, number, number] } | null;
  onViewStateChange?: (state: { position: [number, number, number], target: [number, number, number] }) => void;
  samples?: Sample[];
  onSampleSelect?: (sample: Sample, index: number) => void;
  roadmap?: Roadmap;
  path?: RobotConfiguration[];
  isPathActive?: boolean;
  isExecuting?: boolean;
  onExecutionComplete?: (finalConfig: RobotConfiguration) => void;
  targetPosition?: { x: number, y: number, z: number } | null;
  onTargetPositionChange?: (pos: { x: number, y: number, z: number }) => void;
  isTargetSelected?: boolean;
  onTargetSelect?: (selected: boolean) => void;
}

// 1. Robot Subcomponent
const Robot: React.FC<{ 
  config: RobotConfiguration;
  path?: RobotConfiguration[];
  isExecuting?: boolean;
  onExecutionComplete?: (finalConfig: RobotConfiguration) => void;
}> = ({ config, path, isExecuting, onExecutionComplete }) => {
  const linkMaterial = useMemo(() => new THREE.MeshLambertMaterial({ color: 0xaaaaaa }), []);
  const rotationalMaterial = useMemo(() => new THREE.MeshLambertMaterial({ color: 0x3498db }), []);
  const translationalMaterial = useMemo(() => new THREE.MeshLambertMaterial({ color: 0x2ecc71 }), []);
  const fixedMaterial = useMemo(() => new THREE.MeshLambertMaterial({ color: 0x555555 }), []);

  const jointRefs = useRef<(THREE.Group | null)[]>([]);

  const animState = useRef({
    active: false,
    pathIndex: 0,
    segmentTime: 0,
    segmentDuration: 0,
    currentBase: [] as RobotConfiguration,
    targetNode: [] as RobotConfiguration
  });

  const setupNextSegment = useCallback(() => {
    const s = animState.current;
    if (!path || s.pathIndex >= path.length) {
      s.active = false;
      onExecutionComplete?.(s.currentBase);
      return;
    }
    s.targetNode = path[s.pathIndex];
    
    let maxDuration = 0.1;
    s.currentBase.forEach((joint, i) => {
      const targetJoint = s.targetNode[i];
      const range = joint.max - joint.min;
      if (range <= 0) return;

      let dist = 0;
      if (joint.type === 'rotational') {
        let diff = targetJoint.theta - joint.theta;
        const isFullRange = range >= (2 * Math.PI - 0.01);
        if (isFullRange) {
          diff = Math.atan2(Math.sin(diff), Math.cos(diff));
        }
        dist = Math.abs(diff);
      } else if (joint.type === 'translational') {
        dist = Math.abs(targetJoint.d - joint.d);
      }

      const maxSpeed = range * 0.1; 
      const duration = dist / maxSpeed;
      if (duration > maxDuration) {
        maxDuration = duration;
      }
    });

    s.segmentDuration = maxDuration;
    s.segmentTime = 0;
    s.pathIndex++;
  }, [path, onExecutionComplete]);

  useEffect(() => {
    if (isExecuting && path && path.length > 0) {
      animState.current = {
        active: true,
        pathIndex: 0,
        segmentTime: 0,
        segmentDuration: 0,
        currentBase: [...config],
        targetNode: path[0]
      };
      setupNextSegment();
    } else {
      animState.current.active = false;
    }
  }, [isExecuting, path, config, setupNextSegment]);

  useFrame((_, delta) => {
    const s = animState.current;
    if (!s.active) return;

    s.segmentTime += delta;
    let t = s.segmentDuration > 0 ? s.segmentTime / s.segmentDuration : 1;
    if (t >= 1) t = 1;

    const interpolated = interpolateConfig(s.currentBase, s.targetNode, t);
    
    interpolated.forEach((joint, i) => {
      const group = jointRefs.current[i];
      if (group) {
        if (joint.type === 'rotational') {
          group.rotation.z = joint.theta;
        } else if (joint.type === 'translational') {
          group.position.z = joint.d;
        }
      }
    });

    if (t >= 1) {
      s.currentBase = s.targetNode;
      setupNextSegment();
    }
  });

  const buildTree = (index: number): React.ReactNode => {
    if (index >= config.length) return null;
    const p = config[index];
    
    let jointMat = fixedMaterial;
    if (p.type === 'rotational') jointMat = rotationalMaterial;
    else if (p.type === 'translational') jointMat = translationalMaterial;

    return (
      <group 
        ref={el => { jointRefs.current[index] = el; }}
        rotation={[0, 0, p.theta]} 
        position={[0, 0, p.d]}
      >
        {p.type === 'rotational' && (
          <mesh rotation={[Math.PI / 2, 0, 0]} material={jointMat}>
            <cylinderGeometry args={[25, 25, 60]} />
          </mesh>
        )}
        {p.type === 'translational' && (
          <mesh material={jointMat}>
            <boxGeometry args={[50, 50, 50]} />
          </mesh>
        )}
        {p.type === 'fixed' && (
          <mesh material={jointMat}>
            <sphereGeometry args={[20, 16, 16]} />
          </mesh>
        )}

        {Math.abs(p.d) > 0.1 && (
          <mesh position={[0, 0, -p.d / 2]} rotation={[Math.PI / 2, 0, 0]} material={linkMaterial}>
            <cylinderGeometry args={[10, 10, Math.abs(p.d)]} />
          </mesh>
        )}
        
        <axesHelper args={[100]} />

        <group position={[p.a, 0, 0]} rotation={[p.alpha, 0, 0]}>
          {Math.abs(p.a) > 0.1 && (
            <mesh position={[-p.a / 2, 0, 0]} rotation={[0, 0, Math.PI / 2]} material={linkMaterial}>
              <cylinderGeometry args={[10, 10, Math.abs(p.a)]} />
            </mesh>
          )}
          
          {buildTree(index + 1)}
        </group>
      </group>
    );
  };

  return <>{buildTree(0)}</>;
};

const SingleObstacle: React.FC<{
  obs: Obstacle;
  i: number;
  isSelected: boolean;
  onObstacleSelect?: (index: number | null) => void;
  onObstaclesChange?: (obstacles: Obstacle[]) => void;
  obstacles: Obstacle[];
  orbitControlsRef: React.MutableRefObject<any>;
}> = ({ obs, i, isSelected, onObstacleSelect, onObstaclesChange, obstacles, orbitControlsRef }) => {
  const [meshNode, setMeshNode] = useState<THREE.Mesh | null>(null);

  const handlePointerMissed = (e: React.MouseEvent) => {
    if (e.type === 'click' && isSelected) {
      onObstacleSelect?.(null);
    }
  };

  return (
    <>
      <mesh
        ref={setMeshNode}
        position={[obs.x, obs.y, obs.z]}
        onClick={(e) => {
          e.stopPropagation();
          onObstacleSelect?.(i);
        }}
        onPointerMissed={handlePointerMissed as any}
      >
        <sphereGeometry args={[obs.radius, 32, 32]} />
        <meshLambertMaterial color={0xff5555} transparent opacity={0.7} />
      </mesh>
      
      {isSelected && meshNode && (
        <TransformControls
          object={meshNode}
          mode="translate"
          onMouseDown={() => {
            if (orbitControlsRef.current) orbitControlsRef.current.enabled = false;
          }}
          onMouseUp={() => {
            if (orbitControlsRef.current) orbitControlsRef.current.enabled = true;
          }}
          onObjectChange={(e: any) => {
            if (e?.target?.object && onObstaclesChange) {
              const pos = e.target.object.position;
              if (pos.x !== obs.x || pos.y !== obs.y || pos.z !== obs.z) {
                const newObstacles = [...obstacles];
                newObstacles[i] = {
                  ...newObstacles[i],
                  x: pos.x,
                  y: pos.y,
                  z: pos.z,
                };
                onObstaclesChange(newObstacles);
              }
            }
          }}
        />
      )}
    </>
  );
};

// 2. Obstacles Subcomponent
const Obstacles: React.FC<{
  obstacles: Obstacle[];
  selectedObstacleIndex?: number | null;
  onObstacleSelect?: (index: number | null) => void;
  onObstaclesChange?: (obstacles: Obstacle[]) => void;
  orbitControlsRef: React.MutableRefObject<any>;
}> = ({ obstacles, selectedObstacleIndex, onObstacleSelect, onObstaclesChange, orbitControlsRef }) => {
  return (
    <group>
      {obstacles.map((obs, i) => (
        <SingleObstacle
          key={i}
          obs={obs}
          i={i}
          isSelected={i === selectedObstacleIndex}
          onObstacleSelect={onObstacleSelect}
          onObstaclesChange={onObstaclesChange}
          obstacles={obstacles}
          orbitControlsRef={orbitControlsRef}
        />
      ))}
    </group>
  );
};

// 3. Samples Subcomponent
const Samples: React.FC<{
  samples: Sample[];
  isPathActive: boolean;
  onSampleSelect?: (sample: Sample, index: number) => void;
  onObstacleSelect?: (index: number | null) => void;
}> = ({ samples, isPathActive, onSampleSelect, onObstacleSelect }) => {
  
  const sampleData = useMemo(() => {
    if (isPathActive) return [];
    return samples
      .map((sample, i) => {
        const result = get_end_effector_position(sample.params, []) as KinematicsResult;
        return { index: i, pos: result.ee_position as [number, number, number], sample };
      })
      .filter((s): s is { index: number; pos: [number, number, number]; sample: Sample } => s !== null);
  }, [samples, isPathActive]);

  return (
    <group>
      {sampleData.map((data) => (
        <mesh 
          key={data.index} 
          position={[data.pos[0], data.pos[1], data.pos[2]]}
          onClick={(e) => {
            e.stopPropagation();
            onSampleSelect?.(data.sample, data.index);
            onObstacleSelect?.(null); // Detach obstacle gizmo if selecting sample
            // Target is deselected by its own onPointerMissed when clicking a sample (unless sample stops propagation)
            // But we already have propagation stopped here. So let's add onTargetSelect from props if we wanted to.
            // Actually, just let App handle it, or we add onTargetSelect to Samples.
            // To keep it simple, it's better to rely on App state. 

          }}
        >
          <sphereGeometry args={[15, 16, 16]} />
          <meshLambertMaterial color={0x3498db} transparent opacity={0.8} />
        </mesh>
      ))}
    </group>
  );
};

// 4. Roadmap Subcomponent
const RoadmapVisual: React.FC<{
  roadmap: Roadmap;
  samples: Sample[];
  isPathActive: boolean;
}> = ({ roadmap, samples, isPathActive }) => {
  const linePoints = useMemo(() => {
    if (!roadmap || roadmap.length === 0 || samples.length === 0 || isPathActive) return [];
    
    const points: [number, number, number][] = [];
    const processedEdges = new Set<string>();

    roadmap.forEach((neighbors, i) => {
      const sample1 = samples[i];
      if (!sample1) return;
      const res1 = get_end_effector_position(sample1.params, []) as KinematicsResult;
      
      neighbors.forEach(j => {
        const edgeKey = i < j ? `${i}-${j}` : `${j}-${i}`;
        if (processedEdges.has(edgeKey)) return;
        processedEdges.add(edgeKey);

        const sample2 = samples[j];
        if (!sample2) return;
        const res2 = get_end_effector_position(sample2.params, []) as KinematicsResult;
        
        points.push(
          [res1.ee_position[0], res1.ee_position[1], res1.ee_position[2]],
          [res2.ee_position[0], res2.ee_position[1], res2.ee_position[2]]
        );
      });
    });
    
    return points;
  }, [roadmap, samples, isPathActive]);

  if (linePoints.length === 0) return null;

  return (
    <Line
      points={linePoints}
      color={0x3498db}
      transparent
      opacity={0.3}
      lineWidth={1}
      segments
      depthWrite={false}
    />
  );
};

// 5. Path Subcomponent
const PathVisual: React.FC<{
  path: RobotConfiguration[];
}> = ({ path }) => {
  const linePoints = useMemo(() => {
    if (!path || path.length < 2) return [];
    
    const points: [number, number, number][] = [];
    const segmentsPerStep = 20;
    
    for (let i = 0; i < path.length - 1; i++) {
      const startConfig = path[i];
      const endConfig = path[i + 1];
      
      for (let j = 0; j <= segmentsPerStep; j++) {
        // Skip the last point of the segment unless it's the very last segment
        // to avoid duplicate points in the array
        if (j === segmentsPerStep && i < path.length - 2) continue;

        const t = j / segmentsPerStep;
        const interpolatedConfig = interpolateConfig(startConfig, endConfig, t);
        const res = get_end_effector_position(interpolatedConfig, []) as KinematicsResult;
        points.push([res.ee_position[0], res.ee_position[1], res.ee_position[2]]);
      }
    }
    return points;
  }, [path]);

  if (linePoints.length < 2) return null;

  return (
    <Line
      points={linePoints}
      color={0xff0000}
      lineWidth={3}
      depthWrite={false}
      renderOrder={999}
    />
  );
};

// 6. Target Subcomponent
const Target: React.FC<{
  targetPosition: { x: number, y: number, z: number } | null;
  isTargetSelected?: boolean;
  onTargetSelect?: (selected: boolean) => void;
  onTargetPositionChange?: (pos: { x: number, y: number, z: number }) => void;
  orbitControlsRef: React.MutableRefObject<any>;
}> = ({ targetPosition, isTargetSelected, onTargetSelect, onTargetPositionChange, orbitControlsRef }) => {
  const [meshNode, setMeshNode] = useState<THREE.Mesh | null>(null);
  
  if (!targetPosition) return null;

  const handlePointerMissed = (e: React.MouseEvent) => {
    if (e.type === 'click') {
      onTargetSelect?.(false);
    }
  };

  return (
    <>
      <mesh
        ref={setMeshNode}
        position={[targetPosition.x, targetPosition.y, targetPosition.z]}
        onClick={(e) => {
          e.stopPropagation();
          onTargetSelect?.(true);
        }}
        onPointerMissed={handlePointerMissed as any}
      >
        <sphereGeometry args={[30, 16, 16]} />
        <meshBasicMaterial color={0x27ae60} transparent opacity={0.8} />
      </mesh>
      
      {isTargetSelected && meshNode && (
        <TransformControls
          object={meshNode}
          mode="translate"
          onMouseDown={() => {
            if (orbitControlsRef.current) orbitControlsRef.current.enabled = false;
          }}
          onMouseUp={() => {
            if (orbitControlsRef.current) orbitControlsRef.current.enabled = true;
          }}
          onObjectChange={(e: any) => {
            if (e?.target?.object && onTargetPositionChange) {
              const pos = e.target.object.position;
              if (pos.x !== targetPosition.x || pos.y !== targetPosition.y || pos.z !== targetPosition.z) {
                onTargetPositionChange({ x: pos.x, y: pos.y, z: pos.z });
              }
            }
          }}
        />
      )}
    </>
  );
};

// Controls tracking component to notify parent
const CameraController: React.FC<{
  initialViewState?: { position: [number, number, number], target: [number, number, number] } | null;
  onViewStateChange?: (state: { position: [number, number, number], target: [number, number, number] }) => void;
  orbitControlsRef: React.MutableRefObject<any>;
}> = ({ initialViewState, onViewStateChange, orbitControlsRef }) => {
  const { camera } = useThree();
  const lastNotifyTimeRef = useRef(0);
  const initialized = useRef(false);

  useEffect(() => {
    if (initialized.current) return;
    initialized.current = true;
    
    if (initialViewState) {
      camera.position.set(initialViewState.position[0], initialViewState.position[1], initialViewState.position[2]);
      if (orbitControlsRef.current) {
        orbitControlsRef.current.target.set(initialViewState.target[0], initialViewState.target[1], initialViewState.target[2]);
      }
    } else {
      camera.position.set(1500, -1500, 1500);
    }
    camera.up.set(0, 0, 1);
  }, [initialViewState, camera, orbitControlsRef]);

  const handleChange = () => {
    if (!onViewStateChange || !orbitControlsRef.current) return;
    const now = Date.now();
    if (now - lastNotifyTimeRef.current > 200) {
      onViewStateChange({
        position: [camera.position.x, camera.position.y, camera.position.z],
        target: [
          orbitControlsRef.current.target.x,
          orbitControlsRef.current.target.y,
          orbitControlsRef.current.target.z
        ]
      });
      lastNotifyTimeRef.current = now;
    }
  };

  return (
    <OrbitControls 
      ref={orbitControlsRef} 
      enableDamping 
      makeDefault
      onChange={handleChange}
      target={initialViewState ? initialViewState.target : [0, 0, 0]}
    />
  );
};


// Main Component
const ThreeScene: React.FC<Props> = ({ 
  config, 
  obstacles, 
  onObstaclesChange, 
  selectedObstacleIndex, 
  onObstacleSelect, 
  collision,
  initialViewState,
  onViewStateChange,
  samples = [],
  onSampleSelect,
  roadmap = [],
  path = [],
  isPathActive = false,
  isExecuting = false,
  onExecutionComplete,
  targetPosition = null,
  onTargetPositionChange,
  isTargetSelected,
  onTargetSelect
}) => {
  const orbitControlsRef = useRef<any>(null);

  const bgColor = collision ? '#ffcccc' : '#f0f0f0';

  return (
    <div style={{ width: '100%', height: '100%', position: 'relative', overflow: 'hidden' }}>
      <Canvas camera={{ fov: 50, near: 1, far: 100000, position: [1500, -1500, 1500], up: [0, 0, 1] }}>
        <color attach="background" args={[bgColor]} />
        
        <ambientLight intensity={0.6} />
        <directionalLight position={[1000, 1000, 1000]} intensity={0.8} />

        <CameraController 
          initialViewState={initialViewState} 
          onViewStateChange={onViewStateChange} 
          orbitControlsRef={orbitControlsRef}
        />

        <Robot 
          config={config} 
          path={path}
          isExecuting={isExecuting}
          onExecutionComplete={onExecutionComplete}
        />
        
        <Obstacles 
          obstacles={obstacles} 
          selectedObstacleIndex={selectedObstacleIndex}
          onObstacleSelect={onObstacleSelect}
          onObstaclesChange={onObstaclesChange}
          orbitControlsRef={orbitControlsRef}
        />

        <Samples 
          samples={samples} 
          isPathActive={isPathActive}
          onSampleSelect={onSampleSelect}
          onObstacleSelect={onObstacleSelect}
        />
        
        <RoadmapVisual 
          roadmap={roadmap} 
          samples={samples} 
          isPathActive={isPathActive} 
        />
        
        <PathVisual path={path} />
        
        <Target 
          targetPosition={targetPosition} 
          isTargetSelected={isTargetSelected}
          onTargetSelect={onTargetSelect}
          onTargetPositionChange={onTargetPositionChange}
          orbitControlsRef={orbitControlsRef}
        />

        <gridHelper args={[10000, 100]} rotation={[Math.PI / 2, 0, 0]} />
      </Canvas>
      
      {collision && (
        <div style={{ 
          position: 'absolute', 
          top: 20, 
          left: 20, 
          backgroundColor: 'rgba(220, 53, 69, 0.9)', 
          color: 'white', 
          padding: '5px 15px', 
          borderRadius: '5px',
          fontWeight: 'bold',
          fontSize: '16px',
          zIndex: 1000,
          boxShadow: '0 4px 15px rgba(0,0,0,0.2)',
          pointerEvents: 'none'
        }}>
          <i className="bi bi-exclamation-triangle-fill me-2"></i>
          COLLISION
        </div>
      )}
    </div>
  );
};

export default ThreeScene;
