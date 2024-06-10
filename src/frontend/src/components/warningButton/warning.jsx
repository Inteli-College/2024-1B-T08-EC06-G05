
import EmergencyPopUp from '../emergencyPopUp/emergencyPopUp';
import WarningIcon from './warning.svg';
import { useNavigate } from 'react-router-dom';
import BtVoltarCinza from '../voltarButton/voltar';

const WarningButton = ({ movementhandlers }) => {

  const navigate = useNavigate();

  const mainMenu = () => {
    navigate("../emergencyPopUp");
  };

  const handleClick = () => {
    console.log('Turnoff button pressed');
    if (movementhandlers && movementhandlers.turnoff) {
      movementhandlers.turnoff();
      mainMenu()
    }
  };

  return (
    <button
      onClick={handleClick}
      ontoutch={handleClick}
      className="bg-red-500 flex items-center justify-center p-4 rounded-full text-white active:bg-red-600 focus:outline-none">
      <img src={WarningIcon} alt="Alert" className="h-8 w-8" />
    </button>,
    <BtVoltarCinza />
  );
};

export default WarningButton;

