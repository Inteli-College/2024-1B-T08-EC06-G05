
import WarningIcon from './warning.svg';

const WarningButton = () => {
  return (
    <button className="bg-red-500 flex items-center justify-center p-4 rounded-full text-white active:bg-red-600 focus:outline-none">
      <img src={WarningIcon} alt="Alert" className="h-8 w-8" />
    </button>
  );
};

export default WarningButton;

