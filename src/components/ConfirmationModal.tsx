import React from 'react';

interface Props {
  show: boolean;
  title: string;
  message: string;
  onConfirm: () => void;
  onCancel: () => void;
  confirmText?: string;
  cancelText?: string;
  confirmVariant?: string;
}

const ConfirmationModal: React.FC<Props> = ({
  show,
  title,
  message,
  onConfirm,
  onCancel,
  confirmText = 'Confirm',
  cancelText = 'Cancel',
  confirmVariant = 'primary'
}) => {
  if (!show) return null;

  return (
    <>
      <div className="modal fade show" style={{ display: 'block', backgroundColor: 'rgba(0,0,0,0.5)', zIndex: 1060 }} tabIndex={-1}>
        <div className="modal-dialog modal-dialog-centered">
          <div className="modal-content shadow border-0">
            <div className="modal-header border-0 pb-0">
              <h5 className="modal-title fw-bold">{title}</h5>
              <button type="button" className="btn-close" onClick={onCancel} aria-label="Close"></button>
            </div>
            <div className="modal-body py-4">
              <p className="mb-0 text-muted">{message}</p>
            </div>
            <div className="modal-footer border-0 pt-0">
              <button type="button" className="btn btn-link text-muted text-decoration-none me-auto" onClick={onCancel}>{cancelText}</button>
              <button type="button" className={`btn btn-${confirmVariant} px-4 shadow-sm`} onClick={onConfirm}>{confirmText}</button>
            </div>
          </div>
        </div>
      </div>
      <div className="modal-backdrop fade show" style={{ zIndex: 1050 }}></div>
    </>
  );
};

export default ConfirmationModal;
