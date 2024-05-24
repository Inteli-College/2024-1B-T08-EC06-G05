// src/components/TurnoffButton.js
import React from 'react';
import TurnoffIcon from './Turnoff.svg';

const TurnoffButton = ({ movementhandlers }) => {
  const handleClick = () => {
    console.log('Turnoff button pressed');
    if (movementhandlers && movementhandlers.turnoff) {
      movementhandlers.turnoff();
    }
  };

  return (
    <button
      onClick={handleClick}
      className="bg-red-500 flex items-center justify-center p-2 rounded-full text-white hover:bg-red-600 focus:outline-none"
    >
      <img src={TurnoffIcon} alt="Turn off" className="h-4 w-4" />
    </button>
  );
};

export default TurnoffButton;
