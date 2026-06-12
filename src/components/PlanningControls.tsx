import React, { useState, useEffect } from 'react';
import type { RobotConfiguration, Obstacle, Sample, Roadmap, RoadmapConfig } from '../types/kinematics';
import { find_path_to_target } from '../wasm/prm-compute-pkg/prm_compute';

interface PathResult {
  success: boolean;
  path: RobotConfiguration[] | null;
  error?: string;
}

interface Props {
  robotConfig: RobotConfiguration;
  samples: Sample[];
  roadmap: Roadmap;
  roadmapConfig: RoadmapConfig | null;
  obstacles: Obstacle[];
  onPlanFound: (path: RobotConfiguration[]) => void;
  onExecutePath: (path: RobotConfiguration[]) => void;
  onCancel: () => void;
  hasPath: boolean;
  isExecuting: boolean;
  targetPosition: { x: number; y: number; z: number };
  onTargetChange: (pos: { x: number; y: number; z: number }) => void;
  onError: (message: string) => void;
  heuristicType: string;
  onHeuristicChange: (type: string) => void;
  onTargetFocus?: () => void;
}

const PlanningControls: React.FC<Props> = ({
  robotConfig,
  samples,
  roadmap,
  roadmapConfig,
  obstacles,
  onPlanFound,
  onExecutePath,
  onCancel,
  hasPath,
  isExecuting,
  targetPosition,
  onTargetChange,
  onError,
  heuristicType,
  onHeuristicChange,
  onTargetFocus,
}) => {
  const [isPlanning, setIsPlanning] = useState(false);
  const [numRestarts, setNumRestarts] = useState(10);
  const [localError, setLocalError] = useState<string | null>(null);

  // Clear errors when configuration, samples, or roadmap change
  useEffect(() => {
    setLocalError(null);
  }, [robotConfig, samples, roadmap]);

  const calculateFullPlan = (): RobotConfiguration[] | null => {
    if (samples.length === 0 || roadmap.length === 0 || !roadmapConfig) {
      const msg = 'Requires samples and a generated roadmap.';
      setLocalError(msg);
      onError(msg);
      return null;
    }

    try {
      const result = find_path_to_target(
        robotConfig,
        samples,
        roadmap,
        targetPosition,
        obstacles,
        heuristicType,
        numRestarts,
        roadmapConfig
      ) as PathResult | null;

      if (!result || !result.success || !result.path || result.path.length === 0) {
        const errorMsg = result?.error || 'Could not find a collision-free path to the exact target position.';
        setLocalError(errorMsg);
        onError(errorMsg);
        return null;
      }

      setLocalError(null);
      return result.path;
    } catch (e) {
      console.error(e);
      const errorMsg = 'An error occurred during pathfinding.';
      setLocalError(errorMsg);
      onError(errorMsg);
      return null;
    }
  };

  const handlePlan = () => {
    setIsPlanning(true);
    setLocalError(null);
    onCancel();
    setTimeout(() => {
      const result = calculateFullPlan();
      if (result) {
        onPlanFound(result);
      } else {
        onPlanFound([]);
      }
      setIsPlanning(false);
    }, 100);
  };

  const handlePlanAndExecute = () => {
    setIsPlanning(true);
    setLocalError(null);
    onCancel();
    setTimeout(() => {
      const result = calculateFullPlan();
      if (result) {
        onExecutePath(result);
      } else {
        onPlanFound([]);
      }
      setIsPlanning(false);
    }, 100);
  };

  const handleTargetChange = (newPos: { x: number; y: number; z: number }) => {
    onTargetChange(newPos);
    setLocalError(null);
    if (hasPath && !isExecuting) {
      onCancel();
    }
  };

  const handleHeuristicChange = (type: string) => {
    onHeuristicChange(type);
    setLocalError(null);
  };

  return (
    <div className="planning-controls p-3">
      <div className="row g-2">
        <div className="col-4">
          <label className="form-label small fw-bold">Target X:</label>
          <input
            type="number"
            className="form-control form-control-sm"
            value={Math.round(targetPosition.x)}
            onChange={(e) => handleTargetChange({ ...targetPosition, x: parseInt(e.target.value) || 0 })}
            onFocus={() => onTargetFocus?.()}
            onClick={() => onTargetFocus?.()}
          />
        </div>
        <div className="col-4">
          <label className="form-label small fw-bold">Target Y:</label>
          <input
            type="number"
            className="form-control form-control-sm"
            value={Math.round(targetPosition.y)}
            onChange={(e) => handleTargetChange({ ...targetPosition, y: parseInt(e.target.value) || 0 })}
            onFocus={() => onTargetFocus?.()}
            onClick={() => onTargetFocus?.()}
          />
        </div>
        <div className="col-4">
          <label className="form-label small fw-bold">Target Z:</label>
          <input
            type="number"
            className="form-control form-control-sm"
            value={Math.round(targetPosition.z)}
            onChange={(e) => handleTargetChange({ ...targetPosition, z: parseInt(e.target.value) || 0 })}
            onFocus={() => onTargetFocus?.()}
            onClick={() => onTargetFocus?.()}
          />
        </div>
      </div>

      <div className="row g-2 mt-2">
        <div className="col-12">
          <label className="form-label small fw-bold">Inverse Kinematics Random Restarts:</label>
          <input
            type="number"
            className="form-control form-control-sm"
            value={numRestarts}
            onChange={(e) => setNumRestarts(parseInt(e.target.value) || 1)}
            min="1"
            max="100"
          />
        </div>
      </div>

      <div className="row g-2 mt-2">
        <div className="col-12">
          <label className="form-label small fw-bold">A* Heuristic:</label>
          <select 
            className="form-select form-select-sm" 
            value={heuristicType} 
            onChange={(e) => handleHeuristicChange(e.target.value)}
          >
            <option value="joint-space">Normalized Joint Space Euclidean</option>
            <option value="dijkstra">No heuristic (Dijkstra)</option>
          </select>
        </div>
      </div>

      <div className="row g-2 mt-3">
        {hasPath && !isExecuting ? (
          <>
            <div className="col-6">
              <button
                className="btn btn-success w-100"
                onClick={() => onExecutePath([])} // If path exists, [] triggers execution of existing path
                disabled={isPlanning}
              >
                <i className="bi bi-play-fill me-1"></i> Execute
              </button>
            </div>
            <div className="col-6">
              <button
                className="btn btn-outline-secondary w-100"
                onClick={onCancel}
                disabled={isPlanning}
              >
                <i className="bi bi-x-circle me-1"></i> Cancel
              </button>
            </div>
          </>
        ) : (
          <>
            <div className="col-6">
              <button
                className="btn btn-primary w-100"
                onClick={handlePlan}
                disabled={isPlanning || isExecuting || samples.length === 0 || roadmap.length === 0 || !roadmapConfig}
              >
                {isPlanning ? (
                  <span className="spinner-border spinner-border-sm" role="status"></span>
                ) : (
                  <><i className="bi bi-search me-1"></i> Plan</>
                )}
              </button>
            </div>
            <div className="col-6">
              <button
                className="btn btn-success w-100"
                onClick={handlePlanAndExecute}
                disabled={isPlanning || isExecuting || samples.length === 0 || roadmap.length === 0 || !roadmapConfig}
              >
                <i className="bi bi-play-fill me-1"></i> Plan & Execute
              </button>
            </div>
          </>
        )}
      </div>

      {(samples.length === 0 || roadmap.length === 0 || !roadmapConfig) && (
        <div className="text-danger small mt-2 text-center">
          <i className="bi bi-exclamation-circle me-1"></i>
          Requires samples and roadmap.
        </div>
      )}

      {localError && (
        <div className="alert alert-danger d-flex align-items-center mt-3 p-2 small mb-0" role="alert">
          <i className="bi bi-exclamation-triangle-fill me-2"></i>
          <div>{localError}</div>
        </div>
      )}
    </div>
  );
};

export default PlanningControls;
