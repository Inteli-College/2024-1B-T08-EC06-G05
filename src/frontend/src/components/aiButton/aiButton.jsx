import React from 'react';
import { BsStars } from 'react-icons/bs';

function AiButton() {
  return (
    <button className="bg-gray bg-opacity-70 rounded-xl border border-gray-700 flex items-center justify-center"
    style={{ width: '100px', height: '100px', borderRadius: '10px', backgroundColor:'lightgray'}}>
      <BsStars size={70} />
    </button>
  );
};

export default AiButton;
