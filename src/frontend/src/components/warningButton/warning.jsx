import React, { useState } from 'react';
import EmergencyPopUp from '../emergencyPopUp/emergencyPopUp';
import WarningIcon from './warning.svg';
import { useNavigate } from 'react-router-dom';
import BtVoltarCinza from '../voltarButton/voltar';

const WarningButton = ({ movementhandlers }) => {

  const [openPopup, setOpenPopup] = useState(false);

  const HandleRemovePopUp = () => setOpenPopup(false);

  const handleClick = () => {
    console.log('Turnoff button pressed');
    if (movementhandlers && movementhandlers.turnoff) {
      movementhandlers.turnoff();
    }
    setOpenPopup(true);
  };

  return (
    <button
      onClick={handleClick}
      className="bg-red-500 flex items-center justify-center p-4 rounded-full text-white active:bg-red-600 focus:outline-none">
      <img src={WarningIcon} alt="Alert" className="h-8 w-8" />
      <EmergencyPopUp openPopUp={openPopup} closePopUp={HandleRemovePopUp} />
    </button>
  );
};

export default WarningButton;

