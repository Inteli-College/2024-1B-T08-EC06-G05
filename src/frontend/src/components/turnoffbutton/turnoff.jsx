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
      // className="bg-red-500 flex items-center justhover:bg-red-60ify-center p-2 rounded-full text-white 0 focus:outline-none">
      className="bg-black border border-black rounded">
      <img src={TurnoffIcon} alt="Alert" className="h-4 w-4" />
    </button>
  );
};

export default TurnoffButton;
