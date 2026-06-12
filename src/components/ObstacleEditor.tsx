import React from 'react';
import type { Obstacle } from '../types/kinematics';

interface Props {
  obstacles: Obstacle[];
  onChange: (newObstacles: Obstacle[]) => void;
  selectedIndex?: number | null;
  onSelect?: (index: number | null) => void;
}

const ObstacleEditor: React.FC<Props> = ({ obstacles, onChange, selectedIndex, onSelect }) => {
  const updateObstacle = (index: number, field: keyof Obstacle, value: number) => {
    const newObstacles = [...obstacles];
    newObstacles[index] = { ...newObstacles[index], [field]: value };
    onChange(newObstacles);
  };

  const addObstacle = () => {
    const newObstacle: Obstacle = {
      x: 0,
      y: 0,
      z: 0,
      radius: 50
    };
    onChange([...obstacles, newObstacle]);
  };

  const removeObstacle = (index: number) => {
    const newObstacles = obstacles.filter((_, i) => i !== index);
    onChange(newObstacles);
  };

  return (
    <div className="obstacle-editor">
      <div className="table-responsive">
        <table className="table table-sm table-hover align-middle">
          <thead className="table-light">
            <tr>
              <th>#</th>
              <th>X (mm)</th>
              <th>Y (mm)</th>
              <th>Z (mm)</th>
              <th>Radius (mm)</th>
              <th></th>
            </tr>
          </thead>
          <tbody>
            {obstacles.map((obs, i) => (
              <tr 
                key={i}
                className={selectedIndex === i ? 'table-warning' : ''}
                style={{ cursor: 'pointer' }}
                onClick={() => onSelect?.(i)}
              >
                <td>{i + 1}</td>
                <td>
                  <input 
                    type="number" 
                    className="form-control form-control-sm" 
                    value={Math.round(obs.x)} 
                    step="1"
                    onFocus={() => onSelect?.(i)}
                    onChange={(e) => updateObstacle(i, 'x', parseFloat(e.target.value))}
                  />
                </td>
                <td>
                  <input 
                    type="number" 
                    className="form-control form-control-sm" 
                    value={Math.round(obs.y)} 
                    step="1"
                    onFocus={() => onSelect?.(i)}
                    onChange={(e) => updateObstacle(i, 'y', parseFloat(e.target.value))}
                  />
                </td>
                <td>
                  <input 
                    type="number" 
                    className="form-control form-control-sm" 
                    value={Math.round(obs.z)} 
                    step="1"
                    onFocus={() => onSelect?.(i)}
                    onChange={(e) => updateObstacle(i, 'z', parseFloat(e.target.value))}
                  />
                </td>
                <td>
                  <input 
                    type="number" 
                    className="form-control form-control-sm" 
                    value={Math.round(obs.radius)} 
                    step="1"
                    onFocus={() => onSelect?.(i)}
                    onChange={(e) => updateObstacle(i, 'radius', parseFloat(e.target.value))}
                  />
                </td>
                <td>
                  <button 
                    className="btn btn-outline-danger btn-sm border-0" 
                    onClick={(e) => {
                      e.stopPropagation();
                      removeObstacle(i);
                    }}
                    title="Remove obstacle"
                  >
                    <i className="bi bi-trash"></i>
                  </button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <button className="btn btn-primary w-100 mt-2 shadow-sm" onClick={addObstacle}>
        <i className="bi bi-plus-lg me-2"></i> Add New Obstacle
      </button>
    </div>
  );
};

export default ObstacleEditor;
