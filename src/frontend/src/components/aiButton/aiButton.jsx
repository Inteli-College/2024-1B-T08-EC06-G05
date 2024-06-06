import React from 'react';
import { BsStars } from 'react-icons/bs';

function AiButton() {
  return (
    <button className="bg-white bg-opacity-70 active:bg-opacity-100 rounded-xl border border-gray-700 flex items-center justify-center p-6">
      <BsStars size={70} />
    </button>
  );
};

export default AiButton;
