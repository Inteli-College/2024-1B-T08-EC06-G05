// src/components/StopButton.js
import React from 'react';

const StopButton = ({ movementHandlers }) => {
  const handleClick = () => {
    if (movementHandlers && movementHandlers.stop) {
      movementHandlers.stop();
    }
  };

  return (
    <button
      onClick={handleClick}
      className="bg-red-500 text-white p-2 rounded"
    >
      Stop
    </button>
  );
};

export default StopButton;