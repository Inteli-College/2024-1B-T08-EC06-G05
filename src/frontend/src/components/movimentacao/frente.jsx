// src/components/MoveForward.js
import React from 'react';

const MoveForward = ({ movementhandlers }) => {
  const handleMouseDown = () => {
    console.log('Forward button pressed');
    if (movementhandlers && movementhandlers.forward) {
      movementhandlers.forward();
    }
  };

  const handleMouseUp = () => {
    if (movementhandlers && movementhandlers.stop) {
      movementhandlers.stop();
    }
  };

  return (
    <button
      onMouseDown={handleMouseDown}
      onMouseUp={handleMouseUp}
      onTouchStart={handleMouseDown}
      onTouchEnd={handleMouseUp}
      className="bg-white border border-black rounded p-2"
    >
      <svg width="20" height="20" viewBox="0 0 20 20" fill="black" style={{ transform: 'rotate(-90deg)' }}>
        <polygon points="5,5 15,10 5,15" />
      </svg>
    </button>
  );
};

export default MoveForward;
