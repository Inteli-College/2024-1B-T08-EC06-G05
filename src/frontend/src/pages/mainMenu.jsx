import React from 'react';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faCircleInfo } from '@fortawesome/free-solid-svg-icons';
import SugarZ3roLogo from '../assets/logo-sugarz3ro.svg';
import { useNavigate } from 'react-router-dom'

function MainMenu() {
  const navigate = useNavigate()

const infoPage = () => {
  navigate("/helpScreen")
}
  return (
    <div className="bg-black h-screen flex flex-col items-center justify-center">
      <div className="mb-8">
        <img src={SugarZ3roLogo} alt="Logo SugarZ3ro" />
      </div>
      <div className='flex flex-col items-center space-y-4 md:flex-row md:space-x-10 md:mt-28'>
        <button className="bg-white text-black py-3 px-10 rounded-full font-bold hover:bg-gray-400 mt-2 md:mt-0">Start</button>
        <button className="bg-transparent text-white border-none hover:text-slate-400 mt-4 md:mt-0" style={{ borderRadius: '100%', borderWidth: '2px'}}>
            <FontAwesomeIcon onClick={() => infoPage()}icon={faCircleInfo} style={{ fontSize: '2.5rem'}}/>
        </button>
      </div>
    </div>
  );
};

export default MainMenu;
