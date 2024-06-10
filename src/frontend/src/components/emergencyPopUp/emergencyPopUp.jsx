// src/components/emergencyPopUp/emergencyPopUp.js
import React from 'react';

const EmergencyPopUp = () => {
  return (
    <div className="fixed top-0 left-1/2 transform -translate-x-1/2 z-50">
      <div className="bg-white w-96 p-6 mt-4 rounded-xl shadow-lg">
        <div className="flex flex-col text-center">
          <h1>Tecla de emergência ativada!!</h1>
          <h2>Para voltar a teleoperar, atualize a página</h2>
        </div>
      </div>
    </div>
  );
};

export default EmergencyPopUp;
