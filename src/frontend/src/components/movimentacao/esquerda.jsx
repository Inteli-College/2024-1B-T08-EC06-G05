import React from 'react';

const MovEsquerda = () => {
  return (
    <button className="bg-white border border-black rounded p-2 rotate-180">
      <svg width="20" height="20" viewBox="0 0 20 20" fill="black">
        <polygon points="5,15 15,10 5,5"/>
      </svg>
    </button>
  );
};

export default MovEsquerda;
