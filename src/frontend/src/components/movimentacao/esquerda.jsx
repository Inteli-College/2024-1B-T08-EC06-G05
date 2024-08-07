// src/components/MoveLeft.js
import React from 'react';

const MoveLeft = ({ movementhandlers }) => {
  const handleMouseDown = () => {
    console.log('Left button pressed');
    console.log(movementhandlers)
    if (movementhandlers && movementhandlers.left) {
      console.log('Left button sent');
      movementhandlers.left();
    } else {
      console.log('movementhandlers.left is not defined');
    }
  };

  const handleMouseUp = () => {
    if (movementhandlers && movementhandlers.stop) {
      console.log('Left button released');
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
      <svg width="40" height="40" viewBox="0 0 20 20" fill="black" style={{ transform: 'rotate(180deg)' }}>
        <polygon points="5,5 15,10 5,15" />
      </svg>
    </button>
  );
};

export default MoveLeft;
