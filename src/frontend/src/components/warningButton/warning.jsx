import React from 'react';
import WarningIcon from './warning.svg';
import { Alert } from "@material-tailwind/react";

const WarningButton = ({ movementhandlers, handleAlert }) => {

  const handleClick = () => {
    console.log('Turnoff button pressed');
    handleAlert();
    if (movementhandlers && movementhandlers.turnoff) {
      movementhandlers.turnoff();
    }
  };

  return (
    <button
      onClick={handleClick}
      className="bg-red-500 flex items-center justify-center p-4 rounded-full text-white active:bg-red-600 focus:outline-none">
      <img src={WarningIcon} alt="Alert" className="h-8 w-8" />
    </button>
  );
};

export default WarningButton;