// src/components/MoveRight.js
import React from 'react';

const MoveRight = ({ movementhandlers }) => {
  const handleMouseDown = () => {
    console.log('Right button pressed');
    if (movementhandlers && movementhandlers.right) {
      movementhandlers.right();
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
      className="bg-white border border-black rounded p-2 active:bg-slate-400"
    >
      <svg width="40" height="40" viewBox="0 0 20 20" fill="black">
        <polygon points="5,5 15,10 5,15" />
      </svg>
    </button>
  );
};

export default MoveRight;
