// src/components/TurnoffButton.js
import React from 'react';
import TurnoffIcon from './Turnoff.svg';
import { useNavigate } from 'react-router-dom';


const TurnOffButton = ({ onClick }) => {
  return (
    <button
      onClick={onClick}
      className="bg-red-500 flex items-center justhover:bg-red-60ify-center p-4 rounded-full text-white 0 focus:outline-none active:bg-red-600">
    <img src={TurnoffIcon} alt="Alert" className="h-6 w-6" />
    </button>
  );
};

export default TurnOffButton;
