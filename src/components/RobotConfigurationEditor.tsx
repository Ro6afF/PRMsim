import React from 'react';
import type { RobotConfiguration, DhParameter, JointType } from '../types/kinematics';

interface Props {
  config: RobotConfiguration;
  onChange: (newConfig: RobotConfiguration, isNonVariableChange?: boolean) => void;
}

const RobotConfigurationEditor: React.FC<Props> = ({ config, onChange }) => {
  const radToDeg = (rad: number) => (rad * 180) / Math.PI;
  const degToRad = (deg: number) => (deg * Math.PI) / 180;

  const updateParameter = (index: number, field: keyof DhParameter, value: any) => {
    const newConfig = [...config];
    let newValue = value;

    // Enforce limits for the main joint parameter
    const p = newConfig[index];
    if (field === 'theta' && p.type === 'rotational') {
      newValue = Math.max(p.min, Math.min(p.max, value));
    } else if (field === 'd' && p.type === 'translational') {
      newValue = Math.max(p.min, Math.min(p.max, value));
    } else if (field === 'min') {
      // If min increases above current value, push value up
      if (p.type === 'rotational') {
        if (p.theta < value) newConfig[index].theta = value;
      } else if (p.type === 'translational') {
        if (p.d < value) newConfig[index].d = value;
      }
    } else if (field === 'max') {
      // If max decreases below current value, push value down
      if (p.type === 'rotational') {
        if (p.theta > value) newConfig[index].theta = value;
      } else if (p.type === 'translational') {
        if (p.d > value) newConfig[index].d = value;
      }
    }

    newConfig[index] = { ...newConfig[index], [field]: newValue };

    const isNonVariable = field === 'a' || field === 'alpha' || field === 'type' || field === 'min' || field === 'max' ||
      (field === 'theta' && p.type !== 'rotational') || 
      (field === 'd' && p.type !== 'translational');

    onChange(newConfig, isNonVariable);
  };

  const addLink = () => {
    const newLink: DhParameter = {
      type: 'rotational',
      theta: 0,
      alpha: 0,
      a: 0,
      d: 0,
      min: -Math.PI,
      max: Math.PI
    };
    onChange([...config, newLink], true);
  };

  const removeLink = (index: number) => {
    const newConfig = config.filter((_, i) => i !== index);
    onChange(newConfig, true);
  };

  return (
    <div className="robot-config-editor">
      <div className="table-responsive">
        <table className="table table-sm table-hover align-middle">
          <thead className="table-light">
            <tr>
              <th>#</th>
              <th>a (mm)</th>
              <th>α (deg)</th>
              <th>d (mm)</th>
              <th>θ (deg)</th>
              <th style={{ minWidth: '130px' }}>Type</th>
              <th>Limits (deg/mm)</th>
              <th></th>
            </tr>
          </thead>
          <tbody>
            {config.map((p, i) => (
              <tr key={i}>
                <td>{i + 1}</td>
                <td>
                  <input 
                    type="number" 
                    className="form-control form-control-sm" 
                    value={Math.round(p.a)} 
                    step="1"
                    onChange={(e) => updateParameter(i, 'a', parseFloat(e.target.value))}
                  />
                </td>
                <td>
                  <input 
                    type="number" 
                    className="form-control form-control-sm" 
                    value={Math.round(radToDeg(p.alpha))} 
                    step="1"
                    onChange={(e) => updateParameter(i, 'alpha', degToRad(parseFloat(e.target.value)))}
                  />
                </td>
                <td>
                  <input 
                    type="number" 
                    className={`form-control form-control-sm ${p.type === 'translational' ? 'bg-primary-subtle' : ''}`}
                    value={Math.round(p.d)} 
                    step="1"
                    onChange={(e) => updateParameter(i, 'd', parseFloat(e.target.value))}
                  />
                </td>
                <td>
                  <input 
                    type="number" 
                    className={`form-control form-control-sm ${p.type === 'rotational' ? 'bg-primary-subtle' : ''}`}
                    value={Math.round(radToDeg(p.theta))} 
                    step="1"
                    onChange={(e) => updateParameter(i, 'theta', degToRad(parseFloat(e.target.value)))}
                  />
                </td>
                <td style={{ minWidth: '130px' }}>
                  <select 
                    className="form-select form-select-sm" 
                    value={p.type} 
                    onChange={(e) => updateParameter(i, 'type', e.target.value as JointType)}
                  >
                    <option value="rotational">Rotational</option>
                    <option value="translational">Translational</option>
                    <option value="fixed">Fixed</option>
                  </select>
                </td>
                <td>
                  <div className="input-group input-group-sm">
                    <input 
                      type="number" 
                      className="form-control" 
                      placeholder="Min"
                      step="1"
                      disabled={p.type === 'fixed'}
                      value={p.type === 'rotational' ? Math.round(radToDeg(p.min)) : Math.round(p.min)}
                      onChange={(e) => updateParameter(i, 'min', p.type === 'rotational' ? degToRad(parseFloat(e.target.value)) : parseFloat(e.target.value))}
                    />
                    <input 
                      type="number" 
                      className="form-control" 
                      placeholder="Max"
                      step="1"
                      disabled={p.type === 'fixed'}
                      value={p.type === 'rotational' ? Math.round(radToDeg(p.max)) : Math.round(p.max)}
                      onChange={(e) => updateParameter(i, 'max', p.type === 'rotational' ? degToRad(parseFloat(e.target.value)) : parseFloat(e.target.value))}
                    />
                  </div>
                </td>
                <td>
                  <button 
                    className="btn btn-outline-danger btn-sm border-0" 
                    onClick={() => removeLink(i)}
                    title="Remove link"
                  >
                    <i className="bi bi-trash"></i>
                  </button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <button className="btn btn-primary w-100 mt-2 shadow-sm" onClick={addLink}>
        <i className="bi bi-plus-lg me-2"></i> Add New Link
      </button>
    </div>
  );
};

export default RobotConfigurationEditor;
