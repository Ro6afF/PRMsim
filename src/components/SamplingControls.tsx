import React, { useState } from 'react';
import type { RobotConfiguration, Obstacle, Sample, SamplingStrategy } from '../types/kinematics';
import { generate_samples } from '../wasm/prm-compute-pkg/prm_compute';
import ConfirmationModal from './ConfirmationModal';

interface Props {
  robotConfig: RobotConfiguration;
  obstacles: Obstacle[];
  samples: Sample[];
  onSamplesGenerated: (samples: Sample[]) => void;
  onClearSamples: () => void;
  hasRoadmap: boolean;
  onClearRoadmap: () => void;
}

const SamplingControls: React.FC<Props> = ({ robotConfig, obstacles, samples, onSamplesGenerated, onClearSamples, hasRoadmap, onClearRoadmap }) => {
  const [numSamples, setNumSamples] = useState(1000);
  const [strategy, setStrategy] = useState<SamplingStrategy>('uniform');
  const [sigma, setSigma] = useState(0.05);
  const [isSampling, setIsSampling] = useState(false);
  const [showConfirmAddModal, setShowConfirmAddModal] = useState(false);
  const [showConfirmClearModal, setShowConfirmClearModal] = useState(false);

  const startSampling = () => {
    setIsSampling(true);

    // Use setTimeout to allow UI to update (spinner) before heavy computation
    setTimeout(() => {
      try {
        console.log(`Generating ${numSamples} samples using ${strategy} sampling with sigma=${sigma}...`);
        const startTime = performance.now();

        const result = generate_samples(robotConfig, obstacles, numSamples, strategy, sigma) as Sample[];

        const endTime = performance.now();
        console.log(`Successfully generated ${result.length} valid samples in ${(endTime - startTime).toFixed(2)}ms`);
        onSamplesGenerated(result);
      } catch (error) {
        console.error('Failed to generate samples:', error);
      } finally {
        setIsSampling(false);
      }
    }, 100);
  };

  const handleGenerateSamples = () => {
    if (hasRoadmap) {
      setShowConfirmAddModal(true);
    } else {
      startSampling();
    }
  };

  const handleConfirmAdd = () => {
    setShowConfirmAddModal(false);
    onClearRoadmap();
    startSampling();
  };

  const handleConfirmClear = () => {
    setShowConfirmClearModal(false);
    onClearSamples();
  };

  return (
    <div className="sampling-controls p-3">
      <div className="row g-3">
        <div className="col-12">
          <label htmlFor="samplingStrategy" className="form-label small fw-bold">Sampling Strategy:</label>
          <select 
            id="samplingStrategy" 
            className="form-select" 
            value={strategy} 
            onChange={(e) => {
              const newStrategy = e.target.value as SamplingStrategy;
              setStrategy(newStrategy);
              if (newStrategy === 'gaussian') {
                setSigma(0.05);
              } else if (newStrategy === 'bridge') {
                setSigma(0.15);
              }
            }}
            disabled={isSampling}
          >
            <option value="uniform">Uniform Sampling</option>
            <option value="gaussian">Gaussian Sampling</option>
            <option value="bridge">Bridge Test Sampling</option>
          </select>
        </div>

        {(strategy === 'gaussian' || strategy === 'bridge') && (
          <div className="col-12">
            <label htmlFor="sigma" className="form-label small fw-bold">Sigma (Standard Deviation):</label>
            <input 
              type="number" 
              id="sigma" 
              className="form-control" 
              value={sigma} 
              onChange={(e) => setSigma(parseFloat(e.target.value) || 0)}
              step="0.01"
              min="0.001"
              max="1.0"
              disabled={isSampling}
            />
          </div>
        )}

        <div className="col-12">
          <label htmlFor="numSamples" className="form-label small fw-bold">Number of Samples:</label>
          <input 
            type="number" 
            id="numSamples" 
            className="form-control" 
            value={numSamples} 
            onChange={(e) => setNumSamples(parseInt(e.target.value) || 0)}
            min="1"
            max="5000"
            disabled={isSampling}
          />
        </div>

        <div className="col-12 mt-4">
          <button 
            className="btn btn-success w-100 shadow-sm" 
            onClick={handleGenerateSamples}
            disabled={isSampling}
          >
            {isSampling ? (
              <>
                <span className="spinner-border spinner-border-sm me-2" role="status" aria-hidden="true"></span>
                Sampling...
              </>
            ) : (
              <>
                <i className="bi bi-cpu-fill me-2"></i> Generate Samples
              </>
            )}
          </button>

          <button 
            className="btn btn-outline-danger w-100 mt-2" 
            onClick={() => setShowConfirmClearModal(true)}
            disabled={samples.length === 0 || isSampling}
          >
            <i className="bi bi-trash-fill me-2"></i> Clear Samples
          </button>
        </div>
      </div>
      <div className="mt-3 text-muted small">
        <p className="mb-0">
          <i className="bi bi-info-circle me-1"></i>
          {strategy === 'uniform' && 'Uniformly distributes samples across the joint space.'}
          {strategy === 'gaussian' && 'Concentrates samples near the boundaries of obstacles.'}
          {strategy === 'bridge' && 'Focuses samples in narrow passages between obstacles.'}
        </p>
      </div>

      <ConfirmationModal 
        show={showConfirmAddModal}
        title="Add More Samples?"
        message="Adding new samples will delete the current roadmap. This action cannot be undone. Do you want to continue?"
        confirmText="Clear Roadmap & Add Samples"
        confirmVariant="success"
        onConfirm={handleConfirmAdd}
        onCancel={() => setShowConfirmAddModal(false)}
      />

      <ConfirmationModal 
        show={showConfirmClearModal}
        title="Clear Samples?"
        message="This will delete all generated samples and the current roadmap. This action cannot be undone. Do you want to continue?"
        confirmText="Clear Samples"
        confirmVariant="danger"
        onConfirm={handleConfirmClear}
        onCancel={() => setShowConfirmClearModal(false)}
      />
    </div>
  );
};

export default SamplingControls;
