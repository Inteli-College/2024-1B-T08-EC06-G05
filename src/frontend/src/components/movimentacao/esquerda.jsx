// src/components/MoveLeft.js
import React from 'react';

const MoveLeft = ({ movementHandlers }) => {
  const handleMouseDown = () => {
    if (movementHandlers && movementHandlers.left) {
      movementHandlers.left();
    }
  };

  const handleMouseUp = () => {
    if (movementHandlers && movementHandlers.stop) {
      movementHandlers.stop();
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
      <svg width="20" height="20" viewBox="0 0 20 20" fill="black" style={{ transform: 'rotate(180deg)' }}>
        <polygon points="5,5 15,10 5,15" />
      </svg>
    </button>
  );
};

export default MoveLeft;
