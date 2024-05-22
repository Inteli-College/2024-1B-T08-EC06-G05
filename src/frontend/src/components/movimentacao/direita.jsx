// src/components/MoveRight.js
import React from 'react';

const MoveRight = ({ movementHandlers }) => {
  const handleMouseDown = () => {
    if (movementHandlers && movementHandlers.right) {
      movementHandlers.right();
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
      <svg width="20" height="20" viewBox="0 0 20 20" fill="black">
        <polygon points="5,5 15,10 5,15" />
      </svg>
    </button>
  );
};

export default MoveRight;
