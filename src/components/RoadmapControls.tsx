import React, { useState } from 'react';
import type { Obstacle, Sample, Roadmap, RoadmapConnectingStrategy, RoadmapConfig } from '../types/kinematics';
import { build_prm } from '../wasm/prm-compute-pkg/prm_compute';
import ConfirmationModal from './ConfirmationModal';

interface Props {
  samples: Sample[];
  obstacles: Obstacle[];
  onRoadmapGenerated: (roadmap: Roadmap, config: RoadmapConfig) => void;
  onClearRoadmap: () => void;
  hasRoadmap: boolean;
}

const RoadmapControls: React.FC<Props> = ({ samples, obstacles, onRoadmapGenerated, onClearRoadmap, hasRoadmap }) => {
  const [strategy, setStrategy] = useState<RoadmapConnectingStrategy>('k-nearest');
  const [k, setK] = useState(10);
  const [d, setD] = useState(0.1);
  const [avoidCycles, setAvoidCycles] = useState(false);
  const [isBuilding, setIsBuilding] = useState(false);
  const [showConfirmModal, setShowConfirmModal] = useState(false);

  const handleBuildRoadmap = () => {
    if (samples.length === 0) {
      console.warn('Cannot build roadmap: No samples available.');
      return;
    }

    setIsBuilding(true);

    const config: RoadmapConfig = {
      strategy,
      k,
      d,
      avoidCycles
    };

    // Use setTimeout to allow UI to update (spinner) before heavy computation
    setTimeout(() => {
      try {
        console.log(`Building roadmap using ${strategy} strategy...`, config);
        const startTime = performance.now();

        const result = build_prm(samples, obstacles, config) as { adjacency: Roadmap };

        const endTime = performance.now();
        console.log('Roadmap building result:', result);
        console.log(`Roadmap built in ${(endTime - startTime).toFixed(2)}ms`);
        
        onRoadmapGenerated(result.adjacency, config);
      } catch (error) {
        console.error('Failed to build roadmap:', error);
      } finally {
        setIsBuilding(false);
      }
    }, 100);
  };

  const handleConfirmClear = () => {
    setShowConfirmModal(false);
    onClearRoadmap();
  };

  return (
    <div className="roadmap-controls p-3">
      <div className="row g-3">
        <div className="col-12">
          <label htmlFor="connectingStrategy" className="form-label small fw-bold">Connecting Strategy:</label>
          <select 
            id="connectingStrategy" 
            className="form-select" 
            value={strategy} 
            onChange={(e) => setStrategy(e.target.value as RoadmapConnectingStrategy)}
            disabled={isBuilding}
          >
            <option value="k-nearest">k-Nearest Samples</option>
            <option value="normalized-distance">Normalized distance-based</option>
          </select>
        </div>

        {strategy === 'k-nearest' ? (
          <div className="col-12">
            <label htmlFor="kNeighbors" className="form-label small fw-bold">K Neighbors:</label>
            <input 
              type="number" 
              id="kNeighbors" 
              className="form-control" 
              value={k} 
              onChange={(e) => setK(parseInt(e.target.value) || 0)}
              min="1"
              max="100"
              disabled={isBuilding}
            />
          </div>
        ) : (
          <div className="col-12">
            <label htmlFor="distanceD" className="form-label small fw-bold">Distance (d):</label>
            <input 
              type="number" 
              id="distanceD" 
              className="form-control" 
              value={d} 
              onChange={(e) => setD(parseFloat(e.target.value) || 0)}
              step="0.1"
              min="0.1"
              disabled={isBuilding}
            />
          </div>
        )}

        {strategy === 'normalized-distance' && (
          <div className="col-12 mt-2">
            <div className="alert alert-info py-2 mb-0 small">
              <i className="bi bi-info-circle-fill me-2"></i>
              <strong>Note:</strong> Distances are measured after normalizing each joint to a <code>[0, 1]</code> scale based on its physical limits. This ensures that large-range joints do not dominate the distance calculation.
            </div>
          </div>
        )}

        <div className="col-12">
          <div className="form-check form-switch">
            <input 
              className="form-check-input" 
              type="checkbox" 
              id="avoidCyclesSwitch" 
              checked={avoidCycles}
              onChange={(e) => setAvoidCycles(e.target.checked)}
              disabled={isBuilding}
            />
            <label className="form-check-label small fw-bold" htmlFor="avoidCyclesSwitch">
              Avoid Cycles (Tree structure)
            </label>
          </div>
        </div>

        <div className="col-12 mt-4">
          <button 
            className="btn btn-warning w-100 shadow-sm" 
            onClick={handleBuildRoadmap}
            disabled={isBuilding || samples.length === 0}
          >
            {isBuilding ? (
              <>
                <span className="spinner-border spinner-border-sm me-2" role="status" aria-hidden="true"></span>
                Building Roadmap...
              </>
            ) : (
              <>
                <i className="bi bi-map-fill me-2"></i> Generate Roadmap
              </>
            )}
          </button>
          
          <button 
            className="btn btn-outline-danger w-100 mt-2" 
            onClick={() => setShowConfirmModal(true)}
            disabled={!hasRoadmap}
          >
            <i className="bi bi-trash-fill me-2"></i> Clear Roadmap
          </button>

          {samples.length === 0 && (
            <div className="text-danger small mt-2 text-center">
              <i className="bi bi-exclamation-circle me-1"></i>
              Please generate samples first.
            </div>
          )}
        </div>
      </div>

      <ConfirmationModal 
        show={showConfirmModal}
        title="Clear Roadmap?"
        message="This will delete the current roadmap and any calculated paths, but will keep the samples. This action cannot be undone. Do you want to continue?"
        confirmText="Clear Roadmap"
        confirmVariant="danger"
        onConfirm={handleConfirmClear}
        onCancel={() => setShowConfirmModal(false)}
      />
    </div>
  );
};

export default RoadmapControls;
