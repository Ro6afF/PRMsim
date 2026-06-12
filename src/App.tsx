import { useEffect, useState, useRef, useCallback } from 'react'
import init, { get_end_effector_position, find_path_to_target } from './wasm/prm-compute-pkg/prm_compute'
import type { RobotConfiguration, Obstacle, KinematicsResult, Sample, Roadmap, RoadmapConfig } from './types/kinematics'
import RobotConfigurationEditor from './components/RobotConfigurationEditor'
import ObstacleEditor from './components/ObstacleEditor'
import ThreeScene from './components/ThreeScene'
import SamplingControls from './components/SamplingControls'
import RoadmapControls from './components/RoadmapControls'
import PlanningControls from './components/PlanningControls'
import ConfirmationModal from './components/ConfirmationModal'
import { compressData, decompressData } from './utils/storage'

const DEFAULT_CONFIG: RobotConfiguration = [
  { type: 'rotational', a: 0, alpha: Math.PI / 2, d: 670, theta: 0, min: -Math.PI, max: Math.PI },
  { type: 'rotational', a: 430, alpha: 0, d: 0, theta: 0, min: -Math.PI, max: Math.PI },
  { type: 'rotational', a: 0, alpha: -Math.PI / 2, d: 150, theta: 0, min: -Math.PI, max: Math.PI },
  { type: 'fixed', a: 0, alpha: Math.PI / 2, d: 430, theta: 0, min: 0, max: 0 }
]

const DEFAULT_OBSTACLES: Obstacle[] = [
  { x: 150, y: 70, z: 700, radius: 50 },
  { x: 150, y: -70.01822245717455, z: 700, radius: 50 }
]

const STORAGE_KEY = 'robot-sim-v2-state';

function App() {
  const [wasmReady, setWasmReady] = useState(false)
  const [isInitialized, setIsInitialized] = useState(false)

  const [robotConfig, setRobotConfig] = useState<RobotConfiguration>(DEFAULT_CONFIG)
  const [obstacles, setObstacles] = useState<Obstacle[]>(DEFAULT_OBSTACLES) 
  const [samples, setSamples] = useState<Sample[]>([])
  const [roadmap, setRoadmap] = useState<Roadmap>([])
  const [roadmapConfig, setRoadmapConfig] = useState<RoadmapConfig | null>(null)
  const [targetPosition, setTargetPosition] = useState<{ x: number, y: number, z: number }>({ x: 500, y: 0, z: 500 })
  const [path, setPath] = useState<RobotConfiguration[]>([])
  const [isExecuting, setIsExecuting] = useState(false)
  const [planningError, setPlanningError] = useState<string | null>(null)
  const [selectedObstacleIndex, setSelectedObstacleIndex] = useState<number | null>(null)
  const viewStateRef = useRef<{ position: [number, number, number], target: [number, number, number] } | null>(null)
  const saveTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null)
  const [collision, setCollision] = useState(false)
  const [pendingDhChange, setPendingDhChange] = useState<RobotConfiguration | null>(null)
  const [heuristicType, setHeuristicType] = useState<string>('joint-space')
  const [isTargetSelected, setIsTargetSelected] = useState(false)

  const handleRobotConfigChange = (newConfig: RobotConfiguration, isNonVariableChange?: boolean) => {
    if (isNonVariableChange && samples.length > 0) {
      setPendingDhChange(newConfig);
    } else {
      setRobotConfig(newConfig);
    }
  };

  // Initialization: Load and decompress state
  useEffect(() => {
    const initApp = async () => {
      // 1. Initialize WASM
      try {
        await init();
        setWasmReady(true);
        console.log('WASM initialized');
      } catch (e) {
        console.error('WASM init failed:', e);
      }

      // 2. Load and Decompress Storage
      const saved = localStorage.getItem(STORAGE_KEY);
      if (saved) {
        try {
          let parsed;
          if (saved.startsWith('{')) {
            // Legacy uncompressed format
            parsed = JSON.parse(saved);
          } else {
            // New compressed format
            const decompressed = await decompressData(saved);
            parsed = JSON.parse(decompressed);
          }
          
          if (parsed.robotConfig) setRobotConfig(parsed.robotConfig);
          if (parsed.obstacles) setObstacles(parsed.obstacles);
          if (parsed.samples) setSamples(parsed.samples);
          if (parsed.roadmap) setRoadmap(parsed.roadmap);
          if (parsed.roadmapConfig) setRoadmapConfig(parsed.roadmapConfig);
          if (parsed.targetPosition) setTargetPosition(parsed.targetPosition);
          if (parsed.viewState) viewStateRef.current = parsed.viewState;
        } catch (e) {
          console.error('Failed to restore state:', e);
        }
      }
      setIsInitialized(true);
    };

    initApp();
  }, [])

  // Persistence: Compress and save state
  useEffect(() => {
    if (!isInitialized) return;

    const saveState = async () => {
      try {
        const data = JSON.stringify({ robotConfig, obstacles, viewState: viewStateRef.current, samples, roadmap, roadmapConfig, targetPosition });
        const compressed = await compressData(data);
        localStorage.setItem(STORAGE_KEY, compressed);
      } catch (e) {
        console.error('Failed to save state:', e);
      }
    };

    saveState();
  }, [robotConfig, obstacles, samples, roadmap, roadmapConfig, targetPosition, isInitialized])

  useEffect(() => {
    if (wasmReady && isInitialized) {
      const result = get_end_effector_position(robotConfig, obstacles) as KinematicsResult;
      setCollision(result.collision);
    }
  }, [robotConfig, obstacles, wasmReady, isInitialized])

  useEffect(() => {
    console.log('Current Robot Configuration:', robotConfig)
  }, [robotConfig])

  useEffect(() => {
    console.log('Current Obstacles:', obstacles)
  }, [obstacles])



  const handleExecutionComplete = useCallback((finalConfig: RobotConfiguration) => {
    setIsExecuting(false);
    setPath([]);
    setRobotConfig(finalConfig);
  }, []);

  if (!wasmReady || !isInitialized) {
    return (
      <div className="d-flex justify-content-center align-items-center vh-100">
        <div className="spinner-border text-primary" role="status">
          <span className="visually-hidden">Loading...</span>
        </div>
      </div>
    )
  }

  const handleSampleSelect = (sample: Sample, index: number) => {
    // 1. Update target position coordinates to match the sample
    let targetPos = { x: 0, y: 0, z: 0 };
    try {
      const res = get_end_effector_position(sample.params, []) as KinematicsResult;
      targetPos = {
        x: Math.round(res.ee_position[0]),
        y: Math.round(res.ee_position[1]),
        z: Math.round(res.ee_position[2])
      };
      setTargetPosition(targetPos);
    } catch (error) {
      console.error('Error calculating sample position:', error);
      return;
    }

    // 2. Find path to the sample
    if (roadmap.length > 0 && roadmapConfig) {
      try {
        console.log('Searching for path to sample index:', index);
        const startTime = performance.now();
        const result = find_path_to_target(robotConfig, samples, roadmap, targetPos, obstacles, heuristicType, 1, roadmapConfig) as { success: boolean; path: RobotConfiguration[] | null; error?: string } | null;
        const endTime = performance.now();
        
        if (result && result.success && result.path && result.path.length > 0) {
          console.log(`Path found with ${result.path.length} nodes in ${(endTime - startTime).toFixed(2)}ms`);
          setPath(result.path);
          setIsExecuting(false);
        } else {
          const errorMsg = result?.error || 'No path found to the selected sample.';
          console.warn(errorMsg);
          setPlanningError(errorMsg);
          setPath([]);
          setIsExecuting(false);
        }
      } catch (error) {
        console.error('Error finding path:', error);
        setPath([]);
        setIsExecuting(false);
      }
    } else {
      setPath([sample.params]);
      setIsExecuting(false);
    }
  };

  const handleObstacleSelect = (index: number | null) => {
    setSelectedObstacleIndex(index);
    if (index !== null) {
      setIsTargetSelected(false);
    }
  };

  const handleTargetSelect = (selected: boolean) => {
    setIsTargetSelected(selected);
    if (selected) {
      setSelectedObstacleIndex(null);
    }
  };

  return (
    <div className="container-fluid vh-100 d-flex flex-column p-0 overflow-hidden">
      <div className="row g-0 flex-grow-1 overflow-hidden">
        {/* Left Part: 3D Visualization */}
        <div className="col-md-6 bg-light border-end position-relative h-100">
          <ThreeScene 
            config={robotConfig} 
            obstacles={obstacles} 
            onObstaclesChange={setObstacles} 
            selectedObstacleIndex={selectedObstacleIndex}
            onObstacleSelect={handleObstacleSelect}
            collision={collision}
            initialViewState={viewStateRef.current}
            onViewStateChange={(state) => {
              viewStateRef.current = state;
              if (saveTimeoutRef.current) clearTimeout(saveTimeoutRef.current);
              saveTimeoutRef.current = setTimeout(async () => {
                try {
                  const data = JSON.stringify({ robotConfig, obstacles, viewState: state, samples, roadmap, roadmapConfig, targetPosition });
                  const compressed = await compressData(data);
                  localStorage.setItem(STORAGE_KEY, compressed);
                } catch (e) {
                  console.error('Failed to save view state:', e);
                }
              }, 1000);
            }}
            samples={samples}
            onSampleSelect={handleSampleSelect}
            roadmap={roadmap}
            path={path}
            isPathActive={path.length > 0}
            isExecuting={isExecuting}
            onExecutionComplete={handleExecutionComplete}
            targetPosition={targetPosition}
            onTargetPositionChange={setTargetPosition}
            isTargetSelected={isTargetSelected}
            onTargetSelect={handleTargetSelect}
          />
        </div>

        {/* Right Part: Controls */}
        <div className="col-md-6 d-flex flex-column bg-white h-100 shadow-sm overflow-auto p-4">
          <div className="accordion" id="controlsAccordion">
            {/* DH Parameters Section */}
            <div className="accordion-item border-0 shadow-sm mb-3">
              <h2 className="accordion-header">
                <button className="accordion-button fw-bold" type="button" data-bs-toggle="collapse" data-bs-target="#collapseDH" aria-expanded="true" aria-controls="collapseDH">
                  <i className="bi bi-cpu me-2"></i> DH Parameters
                </button>
              </h2>
              <div id="collapseDH" className="accordion-collapse collapse show">
                <div className="accordion-body p-0">
                  <RobotConfigurationEditor 
                    config={robotConfig} 
                    onChange={handleRobotConfigChange} 
                  />
                </div>
              </div>
            </div>

            {/* Obstacles Section */}
            <div className="accordion-item border-0 shadow-sm mb-3">
              <h2 className="accordion-header">
                <button className="accordion-button fw-bold" type="button" data-bs-toggle="collapse" data-bs-target="#collapseObstacles" aria-expanded="true" aria-controls="collapseObstacles">
                  <i className="bi bi-exclamation-triangle me-2"></i> Obstacles
                </button>
              </h2>
              <div id="collapseObstacles" className="accordion-collapse collapse show">
                <div className="accordion-body p-0">
                  <ObstacleEditor 
                    obstacles={obstacles} 
                    onChange={setObstacles} 
                    selectedIndex={selectedObstacleIndex}
                    onSelect={handleObstacleSelect}
                  />
                </div>
              </div>
            </div>

            {/* Sampling Section */}
            <div className="accordion-item border-0 shadow-sm mb-3">
              <h2 className="accordion-header">
                <button className="accordion-button fw-bold" type="button" data-bs-toggle="collapse" data-bs-target="#collapseSampling" aria-expanded="true" aria-controls="collapseSampling">
                  <i className="bi bi-graph-up me-2"></i> Sampling
                </button>
              </h2>
              <div id="collapseSampling" className="accordion-collapse collapse show">
                <div className="accordion-body p-0">
                  <SamplingControls 
                    robotConfig={robotConfig} 
                    obstacles={obstacles} 
                    samples={samples}
                    onSamplesGenerated={(newSamples) => setSamples(prev => [...prev, ...newSamples])}
                    onClearSamples={() => {
                      setSamples([]);
                      setRoadmap([]);
                      setRoadmapConfig(null);
                      setPath([]);
                    }}
                    hasRoadmap={roadmap.length > 0}
                    onClearRoadmap={() => {
                      setRoadmap([]);
                      setRoadmapConfig(null);
                      setPath([]);
                    }}
                  />
                </div>
              </div>
            </div>

            {/* Roadmap Section */}
            <div className="accordion-item border-0 shadow-sm mb-3">
              <h2 className="accordion-header">
                <button className="accordion-button fw-bold" type="button" data-bs-toggle="collapse" data-bs-target="#collapseRoadmap" aria-expanded="true" aria-controls="collapseRoadmap">
                  <i className="bi bi-map me-2"></i> Roadmap
                </button>
              </h2>
              <div id="collapseRoadmap" className="accordion-collapse collapse show">
                <div className="accordion-body p-0">
                  <RoadmapControls 
                    samples={samples} 
                    obstacles={obstacles} 
                    onRoadmapGenerated={(newRoadmap, config) => {
                      setRoadmap(newRoadmap);
                      setRoadmapConfig(config);
                    }}
                    hasRoadmap={roadmap.length > 0}
                    onClearRoadmap={() => {
                      setRoadmap([]);
                      setRoadmapConfig(null);
                      setPath([]);
                    }}
                  />
                </div>
              </div>
            </div>

            {/* Planning Section */}
            <div className="accordion-item border-0 shadow-sm mb-3">
              <h2 className="accordion-header">
                <button className="accordion-button fw-bold" type="button" data-bs-toggle="collapse" data-bs-target="#collapsePlanning" aria-expanded="true" aria-controls="collapsePlanning">
                  <i className="bi bi-compass me-2"></i> Planning
                </button>
              </h2>
              <div id="collapsePlanning" className="accordion-collapse collapse show">
                <div className="accordion-body p-0">
                  <PlanningControls 
                    robotConfig={robotConfig}
                    samples={samples}
                    roadmap={roadmap}
                    roadmapConfig={roadmapConfig}
                    obstacles={obstacles}
                    targetPosition={targetPosition}
                    onTargetChange={setTargetPosition}
                    heuristicType={heuristicType}
                    onHeuristicChange={setHeuristicType}
                    hasPath={path.length > 0}
                    isExecuting={isExecuting}
                    onPlanFound={(foundPath) => {
                      setPath(foundPath);
                      setIsExecuting(false);
                    }}
                    onExecutePath={(foundPath) => {
                      if (foundPath.length > 0) {
                        setPath(foundPath);
                      }
                      setIsExecuting(true);
                    }}
                    onCancel={() => {
                      setPath([]);
                    }}
                    onError={(msg) => setPlanningError(msg)}
                    onTargetFocus={() => handleTargetSelect(true)}
                  />
                </div>
              </div>
            </div>
          </div>

          <div className="mt-auto pt-3 border-top text-muted small">
            PRM simulation. Made by Dimo Chanev.
          </div>
        </div>
      </div>
      
      <ConfirmationModal 
        show={pendingDhChange !== null}
        title="Warning"
        message="Editing structural DH parameters will clear the currently generated roadmap and samples. Do you want to continue?"
        confirmText="Continue"
        onConfirm={() => {
          if (pendingDhChange) {
            setRobotConfig(pendingDhChange);
            setSamples([]);
            setRoadmap([]);
            setRoadmapConfig(null);
            setPath([]);
            setPendingDhChange(null);
          }
        }}
        onCancel={() => setPendingDhChange(null)}
      />

      <ConfirmationModal 
        show={planningError !== null}
        title="Planning Error"
        message={planningError || ''}
        confirmText="Close"
        onConfirm={() => setPlanningError(null)}
        onCancel={() => setPlanningError(null)}
      />

      <ResetButton />
    </div>
  )
}

const ResetButton = () => {
  const handleReset = () => {
    localStorage.removeItem('robot-sim-v2-state');
    window.location.reload();
  };

  return (
    <>
      <button 
        type="button"
        className="btn btn-danger shadow-lg d-flex align-items-center justify-content-center"
        data-bs-toggle="modal" 
        data-bs-target="#resetModal"
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '45px',
          height: '45px',
          borderRadius: '50%',
          zIndex: 2000,
          opacity: 0.8
        }}
        title="Reset Simulation"
      >
        <i className="bi bi-arrow-counterclockwise fs-5"></i>
      </button>

      {/* Reset Confirmation Modal */}
      <div className="modal fade" id="resetModal" tabIndex={-1} aria-labelledby="resetModalLabel" aria-hidden="true">
        <div className="modal-dialog modal-dialog-centered">
          <div className="modal-content border-0 shadow-lg">
            <div className="modal-header bg-danger text-white">
              <h5 className="modal-title" id="resetModalLabel">
                <i className="bi bi-exclamation-triangle-fill me-2"></i>
                Reset Simulation?
              </h5>
              <button type="button" className="btn-close btn-close-white" data-bs-dismiss="modal" aria-label="Close"></button>
            </div>
            <div className="modal-body p-4">
              <p className="mb-0">This will permanently clear all obstacles, samples, and configurations. You will lose your current progress.</p>
              <p className="mt-2 fw-bold text-danger mb-0">Are you sure you want to proceed?</p>
            </div>
            <div className="modal-footer bg-light">
              <button type="button" className="btn btn-outline-secondary" data-bs-dismiss="modal">Cancel</button>
              <button type="button" className="btn btn-danger px-4" onClick={handleReset}>
                <i className="bi bi-trash-fill me-2"></i>
                Reset Everything
              </button>
            </div>
          </div>
        </div>
      </div>
    </>
  );
};

export default App
