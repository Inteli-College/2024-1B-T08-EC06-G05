import React, { useState } from 'react';
import { BsStars } from 'react-icons/bs';

function AiButton({ onButtonStateChange }) {

  const [buttonState, setButtonState] = useState(false);

  const handleClick = () => {
    const newState = !buttonState;
    setButtonState(newState);
    if (typeof onButtonStateChange === 'function') {
      onButtonStateChange(newState);
    }
  }

  return (
    <button
      className="bg-white bg-opacity-70 active:bg-opacity-100 rounded-xl border border-gray-700 flex items-center justify-center p-6"
      onClick={handleClick}
    >
      <BsStars size={70} />
    </button>
  );
};

export default AiButton;
