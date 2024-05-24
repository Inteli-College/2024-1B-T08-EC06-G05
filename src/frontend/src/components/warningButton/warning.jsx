
import WarningIcon from './warning.svg';

const WarningButton = () => {
  return (
    <button className="bg-red-500 flex items-center justify-center p-2 rounded-full text-white hover:bg-red-600 focus:outline-none">
      <img src={WarningIcon} alt="Alert" className="h-4 w-4" />
    </button>
  );
};

export default WarningButton;

