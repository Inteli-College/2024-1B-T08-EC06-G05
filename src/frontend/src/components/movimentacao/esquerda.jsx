// src/components/MoveLeft.js
import React from 'react';
import TurtleBotController from '../rosbridge_movement/rosbridge_movement';


const MoveLeft = ({ movementhandlers }) => {
  const handleMouseDown = () => {
    if (movementhandlers && movementhandlers.left) {
      movementhandlers.left();
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
      <svg width="20" height="20" viewBox="0 0 20 20" fill="black" style={{ transform: 'rotate(180deg)' }}>
        <polygon points="5,5 15,10 5,15" />
      </svg>
    </button>
  );
};

export default MoveLeft;
